#!/usr/bin/env python3
"""Gemini 336 depth simulation for a U-shaped, single-opening scene."""

import math
import json

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Bool, Header, String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


class Gemini336DepthScene(Node):
    """Render depth, ground truth, map bounds, and observation poses."""

    def __init__(self):
        super().__init__('gemini336_depth_scene')
        self.width = int(self.declare_parameter('camera.width', 640).value)
        self.height = int(self.declare_parameter('camera.height', 480).value)
        self.fx = float(self.declare_parameter('camera.fx', 387.229248).value)
        self.fy = float(self.declare_parameter('camera.fy', 387.229248).value)
        self.cx = float(self.declare_parameter('camera.cx', 321.046387).value)
        self.cy = float(self.declare_parameter('camera.cy', 243.449692).value)
        self.frame_id = str(self.declare_parameter(
            'camera.frame_id', 'gemini336_depth_optical_frame').value)
        self.map_frame = str(self.declare_parameter('map_frame', 'map').value)
        self.rate_hz = float(self.declare_parameter('publish_rate', 5.0).value)
        self.pixel_stride = max(1, int(self.declare_parameter('pixel_stride', 2).value))
        self.min_depth = float(self.declare_parameter('depth.min_range', 0.2).value)
        self.max_depth = float(self.declare_parameter('depth.max_range', 5.0).value)
        self.scenario = str(self.declare_parameter('scenario', 'front_opening').value)
        self.planner_enabled = bool(self.declare_parameter(
            'observation_planner_enabled', False).value)
        self.target_topic = str(self.declare_parameter(
            'observation.target_topic',
            '/complex_area_plan/observation_target').value)
        self.state_topic = str(self.declare_parameter(
            'observation.state_topic',
            '/complex_area_plan/observation_state').value)
        self.frames_per_observation = max(2, int(self.declare_parameter(
            'frames_per_observation', 10).value))
        self.wait_for_fusion = bool(self.declare_parameter(
            'wait_for_fusion', True).value)

        self.boxes = self._scene_boxes()
        self.map_polygon = [(-1.0, -0.9), (1.0, -0.9),
                            (1.0, 0.85), (-1.0, 0.85)]
        all_poses = self._pose_sequence()
        # The planner owns all subsequent observation decisions. The scenario
        # only supplies the initial stationary pose used to seed the map.
        self.pose_sequence = [all_poses[0]] if self.planner_enabled else all_poses
        self.pose_index = 0
        self.frame_in_observation = 0
        self.observed_pose_indices = set()
        self.pose_x, self.pose_y, self.pose_z, self.yaw = self.pose_sequence[0]
        self.observation_points = []
        self.station_ready = False
        self.pending_target = None
        self.observation_complete = False

        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_rect_raw', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '/camera/depth/points', 10)
        latched_qos = QoSProfile(depth=1,
                                 reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.ground_truth_pub = self.create_publisher(
            PointCloud2, '/scene/ground_truth', latched_qos)
        self.scene_marker_pub = self.create_publisher(
            MarkerArray, '/scene/reference_markers', latched_qos)
        task_qos = QoSProfile(depth=1,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.task_pub = self.create_publisher(String, '/area_designation', task_qos)
        self.status_sub = self.create_subscription(
            String, '/grid_map/mapping_status', self.on_mapping_status, task_qos)
        self.target_sub = self.create_subscription(
            PoseStamped, self.target_topic, self.on_observation_target,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.state_sub = self.create_subscription(
            String, self.state_topic, self.on_observation_state,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.wipe_complete_pub = self.create_publisher(
            Bool, '/complex_area_plan/wipe_complete', 10)
        self.wipe_sub = self.create_subscription(
            String, '/complex_area_plan/wipe_request', self.on_wipe_request,
            task_qos)
        self.tf_pub = TransformBroadcaster(self)
        self.publish_ground_truth()
        self.publish_reference_markers()
        self.task_sent = False
        self.task_timer = self.create_timer(1.0, self.publish_task)
        self.timer = self.create_timer(1.0 / max(0.1, self.rate_hz), self.publish_frame)
        self.get_logger().info(
            'scenario=%s planner=%s U opening=0.30m depth=0.50m '
            'first_pose=(%.2f, %.2f, %.2f) yaw=%.1fdeg'
            % (self.scenario, self.planner_enabled, self.pose_x, self.pose_y, self.pose_z,
               math.degrees(self.yaw)))

    def _scene_boxes(self):
        # U opens toward -Y. Inner opening is 0.30 m and cavity depth is 0.50 m.
        wall = 0.08
        inner_half_width = 0.15
        outer_half_width = inner_half_width + wall
        side_center = inner_half_width + wall / 2.0
        return [
            ((-side_center, 0.25, 0.35), (wall / 2.0, 0.25, 0.35)),
            ((side_center, 0.25, 0.35), (wall / 2.0, 0.25, 0.35)),
            ((0.0, 0.46, 0.35), (outer_half_width, wall / 2.0, 0.35)),
            ((0.0, 0.54, 0.75), (0.90, wall / 2.0, 0.75)),
        ]

    def _pose_sequence(self):
        front = (0.0, -0.75, 1.15, 0.0)
        left_side = (-0.65, 0.24, 1.15, -math.pi / 2.0)
        right_side = (0.65, 0.24, 1.15, math.pi / 2.0)
        if self.scenario == 'side_obstacle':
            return [left_side, front, right_side]
        return [front, left_side, right_side]

    def publish_task(self):
        if self.task_sent:
            return
        points = ','.join('{"x":%.3f,"y":%.3f}' % point
                          for point in self.map_polygon)
        message = String()
        message.data = '{"command":"start","frame_id":%s,"points":[%s]}' % (
            json.dumps(self.map_frame), points)
        self.task_pub.publish(message)
        self.task_sent = True

    def on_mapping_status(self, message):
        if not self.wait_for_fusion:
            return
        try:
            status = json.loads(message.data)
            position_index = int(status.get('position_index', 0))
            ready = bool(status.get('ready_to_move', False))
            self.station_ready = ready if self.planner_enabled else \
                ready and position_index >= self.pose_index + 1
        except (TypeError, ValueError, json.JSONDecodeError):
            self.get_logger().warning('invalid mapping status: %s' % message.data)

    def on_observation_target(self, message):
        if not self.planner_enabled or message.header.frame_id not in ('', self.map_frame):
            return
        q = message.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        # Planner PoseStamped yaw uses the conventional +X forward axis;
        # this renderer's camera optical axis maps to +Y at yaw=0.
        yaw -= math.pi / 2.0
        self.pending_target = (float(message.pose.position.x),
                               float(message.pose.position.y),
                               float(message.pose.position.z), yaw)

    def on_observation_state(self, message):
        if self.planner_enabled and message.data in (
                'observation_complete', 'done',
                'observation_aborted_below_wipe_threshold'):
            self.observation_complete = True
            self.get_logger().info('observation planner completed the experiment')

    def on_wipe_request(self, message):
        if not self.planner_enabled:
            return
        try:
            request = json.loads(message.data)
            self.get_logger().info(
                'simulated wipe round %d: %s cells' % (
                    int(request.get('observation_round', 0)),
                    str(request.get('wipeable_cells', 0))))
        except (TypeError, ValueError, json.JSONDecodeError):
            self.get_logger().warning('invalid wipe request: %s' % message.data)
            return
        complete = Bool()
        complete.data = True
        self.wipe_complete_pub.publish(complete)

    def _camera_rotation(self):
        # R_map_from_optical = Rz(yaw) * Rx(-90 deg).
        cz, sz = math.cos(self.yaw / 2.0), math.sin(self.yaw / 2.0)
        sx, cx = -math.sqrt(0.5), math.sqrt(0.5)
        return cz * sx, sz * sx, sz * cx, cz * cx

    def publish_camera_transform(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.frame_id
        transform.transform.translation.x = self.pose_x
        transform.transform.translation.y = self.pose_y
        transform.transform.translation.z = self.pose_z
        (transform.transform.rotation.x, transform.transform.rotation.y,
         transform.transform.rotation.z, transform.transform.rotation.w) = self._camera_rotation()
        self.tf_pub.sendTransform(transform)

    @staticmethod
    def _sample_box_surface(center, half, spacing=0.04):
        center, half = np.asarray(center, dtype=np.float32), np.asarray(half, dtype=np.float32)
        points = []
        for axis in range(3):
            other = [a for a in range(3) if a != axis]
            ranges = [np.arange(-half[a], half[a] + spacing * 0.5, spacing) for a in other]
            for sign in (-1.0, 1.0):
                for u in ranges[0]:
                    for v in ranges[1]:
                        p = center.copy()
                        p[axis] += sign * half[axis]
                        p[other[0]] += u
                        p[other[1]] += v
                        points.append(p)
        return np.asarray(points, dtype=np.float32)

    def publish_ground_truth(self):
        points = np.concatenate([self._sample_box_surface(center, half)
                                 for center, half in self.boxes], axis=0)
        message = PointCloud2()
        message.header.frame_id = self.map_frame
        message.height, message.width = 1, int(points.shape[0])
        message.fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                          PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                          PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]
        message.is_bigendian, message.point_step = False, 12
        message.row_step, message.is_dense = 12 * message.width, True
        message.data = points.astype('<f4', copy=False).tobytes()
        self.ground_truth_pub.publish(message)

    @staticmethod
    def _point(x, y, z):
        point = Point()
        point.x, point.y, point.z = float(x), float(y), float(z)
        return point

    def publish_reference_markers(self):
        markers, stamp = MarkerArray(), self.get_clock().now().to_msg()
        for marker_id, (center, half) in enumerate(self.boxes):
            marker = Marker(header=Header(stamp=stamp, frame_id=self.map_frame),
                            ns='true_scene_u_walls', id=marker_id, type=Marker.CUBE,
                            action=Marker.ADD)
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = center
            marker.pose.orientation.w = 1.0
            marker.scale.x, marker.scale.y, marker.scale.z = tuple(2.0 * v for v in half)
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.9, 0.18, 0.12, 0.30
            markers.markers.append(marker)

        bounds = Marker(header=Header(stamp=stamp, frame_id=self.map_frame),
                        ns='original_map_range', id=100, type=Marker.LINE_STRIP,
                        action=Marker.ADD)
        bounds.scale.x = 0.015
        bounds.color.r, bounds.color.g, bounds.color.b, bounds.color.a = 0.1, 0.95, 0.2, 1.0
        bounds.points = [self._point(x, y, 0.015) for x, y in self.map_polygon]
        bounds.points.append(bounds.points[0])
        markers.markers.append(bounds)

        observations = Marker(header=Header(stamp=stamp, frame_id=self.map_frame),
                              ns='observation_poses', id=200, type=Marker.LINE_STRIP,
                              action=Marker.ADD)
        observations.scale.x = 0.025
        observations.color.r, observations.color.g, observations.color.b, observations.color.a = 0.1, 0.7, 1.0, 0.9
        observations.points = [self._point(x, y, 0.04)
                               for _, x, y in self.observation_points]
        markers.markers.append(observations)
        for observation_id, x, y in self.observation_points:
            dot = Marker(header=Header(stamp=stamp, frame_id=self.map_frame),
                         ns='observation_positions', id=observation_id,
                         type=Marker.SPHERE, action=Marker.ADD)
            dot.pose.position = self._point(x, y, 0.04)
            dot.pose.orientation.w = 1.0
            dot.scale.x = dot.scale.y = dot.scale.z = 0.09
            dot.color.r, dot.color.g, dot.color.b, dot.color.a = 0.05, 0.55, 1.0, 1.0
            markers.markers.append(dot)
            label = Marker(header=Header(stamp=stamp, frame_id=self.map_frame),
                           ns='observation_labels', id=observation_id,
                           type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
            label.pose.position = self._point(x, y, 0.18)
            label.pose.orientation.w = 1.0
            label.scale.z = 0.12
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = 'O%d (%.2f, %.2f)' % (observation_id + 1, x, y)
            markers.markers.append(label)
        self.scene_marker_pub.publish(markers)

    def render_depth(self):
        rows, cols = np.mgrid[0:self.height:self.pixel_stride, 0:self.width:self.pixel_stride]
        x_cam = (cols.astype(np.float32) - self.cx) / self.fx
        y_cam = (rows.astype(np.float32) - self.cy) / self.fy
        # Build rays in the camera optical frame (x right, y down, z forward)
        # and rotate them into the map frame for ray-box intersections.
        optical_dirs = np.stack((x_cam, y_cam, np.ones_like(x_cam)), axis=-1)
        optical_dirs /= np.linalg.norm(optical_dirs, axis=-1, keepdims=True)
        base_dirs = np.stack((optical_dirs[..., 0], optical_dirs[..., 2],
                              -optical_dirs[..., 1]), axis=-1)
        yaw_cos, yaw_sin = math.cos(self.yaw), math.sin(self.yaw)
        dirs = base_dirs.copy()
        dirs[..., 0] = yaw_cos * base_dirs[..., 0] - yaw_sin * base_dirs[..., 1]
        dirs[..., 1] = yaw_sin * base_dirs[..., 0] + yaw_cos * base_dirs[..., 1]
        origin = np.array([self.pose_x, self.pose_y, self.pose_z], dtype=np.float32)
        ray_distance = np.full(rows.shape, np.inf, dtype=np.float32)
        floor_t = -origin[2] / dirs[..., 2]
        ray_distance = np.where((dirs[..., 2] < -1e-5) & (floor_t > 0.0), floor_t, ray_distance)
        for center, half in self.boxes:
            center, half = np.asarray(center), np.asarray(half)
            inverse = np.where(np.abs(dirs) > 1e-7, 1.0 / dirs, np.inf)
            t0, t1 = (center - half - origin) * inverse, (center + half - origin) * inverse
            near, far = np.max(np.minimum(t0, t1), axis=-1), np.min(np.maximum(t0, t1), axis=-1)
            hit = far >= np.maximum(near, 0.0)
            box_t = np.where(near > 0.0, near, far)
            ray_distance = np.where(hit & (box_t > 0.0), np.minimum(ray_distance, box_t), ray_distance)
        # The published depth image stores the optical z component, not the
        # Euclidean distance along the map-frame ray.
        depth = ray_distance * optical_dirs[..., 2]
        return rows, cols, np.where((depth >= self.min_depth) & (depth <= self.max_depth), depth, 0.0)

    def publish_frame(self):
        if self.observation_complete:
            self.timer.cancel()
            return
        if self.planner_enabled and self.pending_target is not None:
            target = self.pending_target
            self.pending_target = None
            self.pose_x, self.pose_y, target_z, self.yaw = target
            self.pose_z = target_z if target_z > 0.2 else self.pose_z
            self.pose_index += 1
            self.frame_in_observation = 0
            self.station_ready = False
            self.get_logger().info(
                'planner observation target=%d pose=(%.2f, %.2f, %.2f) yaw=%.1fdeg'
                % (self.pose_index + 1, self.pose_x, self.pose_y,
                   self.pose_z, math.degrees(self.yaw)))
        ready_to_switch = self.station_ready if self.wait_for_fusion else \
            self.frame_in_observation >= self.frames_per_observation
        if (not self.planner_enabled and self.task_sent and ready_to_switch and
                self.pose_index < len(self.pose_sequence) - 1):
            self.pose_index = min(self.pose_index + 1, len(self.pose_sequence) - 1)
            self.pose_x, self.pose_y, self.pose_z, self.yaw = self.pose_sequence[self.pose_index]
            self.frame_in_observation = 0
            self.station_ready = False
            self.get_logger().info(
                'observation=%d pose=(%.2f, %.2f, %.2f) yaw=%.1fdeg'
                % (self.pose_index + 1, self.pose_x, self.pose_y,
                   self.pose_z, math.degrees(self.yaw)))
        if (self.frame_in_observation == 0 and
                self.pose_index not in self.observed_pose_indices):
            observation_id = len(self.observation_points)
            self.observation_points.append((observation_id, self.pose_x, self.pose_y))
            self.observed_pose_indices.add(self.pose_index)
        if self.task_sent:
            self.frame_in_observation += 1
        stamp = self.get_clock().now().to_msg()
        self.publish_camera_transform()
        rows, cols, depth = self.render_depth()
        header = Header(stamp=stamp, frame_id=self.frame_id)
        full_depth = np.zeros((self.height, self.width), dtype='<f4')
        full_depth[rows, cols] = depth
        self.depth_pub.publish(Image(header=header, height=self.height, width=self.width,
                                     encoding='32FC1', is_bigendian=False,
                                     step=self.width * 4, data=full_depth.tobytes()))
        valid = depth > 0.0
        # Keep the station fusion gate live when a planner-selected pose has
        # no intersected surface in the synthetic scene.  This is a floor
        # clearing return expressed in the optical frame, so it does not add
        # an obstacle to the height map but still provides a valid ray.
        if not np.any(valid):
            floor_map = np.array([
                self.pose_x + 0.5 * math.cos(self.yaw),
                self.pose_y + 0.5 * math.sin(self.yaw), 0.0], dtype=np.float32)
            delta = floor_map - np.array(
                [self.pose_x, self.pose_y, self.pose_z], dtype=np.float32)
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            base_x = c * delta[0] + s * delta[1]
            base_y = -s * delta[0] + c * delta[1]
            points = np.asarray([[base_x, -delta[2], base_y]], dtype='<f4')
        else:
            points = np.stack(((cols[valid].astype(np.float32) - self.cx) / self.fx * depth[valid],
                               (rows[valid].astype(np.float32) - self.cy) / self.fy * depth[valid],
                               depth[valid]), axis=-1).astype('<f4', copy=False)
            floor_map = np.array([
                self.pose_x + 0.5 * math.cos(self.yaw),
                self.pose_y + 0.5 * math.sin(self.yaw), 0.0], dtype=np.float32)
            delta = floor_map - np.array(
                [self.pose_x, self.pose_y, self.pose_z], dtype=np.float32)
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            base_x = c * delta[0] + s * delta[1]
            base_y = -s * delta[0] + c * delta[1]
            points = np.concatenate((points, np.asarray(
                [[base_x, -delta[2], base_y]], dtype='<f4')), axis=0)
        cloud = PointCloud2(header=header, height=1, width=int(points.shape[0]),
                            is_bigendian=False, point_step=12,
                            row_step=12 * int(points.shape[0]), is_dense=True,
                            data=points.tobytes())
        cloud.fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]
        self.cloud_pub.publish(cloud)
        self.publish_reference_markers()


def main():
    rclpy.init()
    node = Gemini336DepthScene()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
