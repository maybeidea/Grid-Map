#!/usr/bin/env python3
"""Compare committed GridMap snapshots with the synthetic scene geometry."""

import csv
import json
import math
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


HEIGHT_VALUE = {0: 0, 25: 1, 50: 2, 75: 3, 100: 4}


class MappingEvaluator(Node):
    def __init__(self):
        super().__init__('gemini336_mapping_evaluator')
        self.output_dir = Path(self.declare_parameter(
            'output_dir', '/tmp/plan_env_gemini336').value)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.boxes = []
        self.heightmap = None
        latched = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_sub = self.create_subscription(
            MarkerArray, '/scene/reference_markers', self.on_markers, latched)
        self.height_sub = self.create_subscription(
            OccupancyGrid, '/grid_map/heightmap', self.on_heightmap, latched)
        self.processed_snapshots = set()
        self.timer = self.create_timer(1.0, self.evaluate_new_snapshots)
        self.get_logger().info('writing metrics to %s' % self.output_dir)

    def on_markers(self, message):
        boxes = []
        for marker in message.markers:
            if marker.ns != 'true_scene_u_walls' or marker.action == Marker.DELETE:
                continue
            center = marker.pose.position
            scale = marker.scale
            boxes.append((np.array([center.x, center.y, center.z]),
                          np.array([scale.x, scale.y, scale.z]) * 0.5))
        if boxes:
            self.boxes = boxes

    def on_heightmap(self, message):
        self.heightmap = message

    def evaluate_new_snapshots(self):
        if not self.boxes or self.heightmap is None:
            return
        for path in sorted(self.output_dir.glob('map_v*_esdf3d.json')):
            if path.name in self.processed_snapshots:
                continue
            try:
                data = json.loads(path.read_text())
                version = int(data['version'])
                metrics = self.evaluate(data, path)
                if metrics is None:
                    if self.has_no_known_occupancy(data):
                        self.processed_snapshots.add(path.name)
                    continue
                metrics['version'] = version
                metrics['snapshot'] = path.name
                self.write_metrics(metrics)
                self.processed_snapshots.add(path.name)
                self.get_logger().info(
                    'v%d IoU=%.4f free_precision=%.4f height_accuracy=%.4f '
                    'coverage=%.4f ESDF_MAE=%.4fm' % (
                        version, metrics['occupied_iou'],
                        metrics['free_space_precision'],
                        metrics['heightmap_accuracy'],
                        metrics['heightmap_coverage'], metrics['esdf_mae_m']))
            except (KeyError, TypeError, ValueError, OSError) as error:
                self.get_logger().warning('cannot evaluate %s: %s' % (path, error))

    @staticmethod
    def has_no_known_occupancy(data):
        states = data.get('occupancy_state', '')
        return bool(states) and all(state == '2' for state in states)

    def evaluate(self, data, path):
        dimensions = tuple(int(value) for value in data['dimensions'])
        nx, ny, nz = dimensions
        resolution = float(data['resolution'])
        origin = np.asarray(data['origin'], dtype=np.float64)
        total = nx * ny * nz
        state_text = data['occupancy_state']
        predicted = np.frombuffer(state_text.encode('ascii'), dtype=np.uint8) - ord('0')
        distance_path = path.parent / data['esdf_distance_file']
        distances = np.fromfile(distance_path, dtype='<f4').astype(np.float64)
        if predicted.size != total or distances.size != total:
            return None

        linear = np.arange(total, dtype=np.int64)
        z_id = linear % nz
        y_id = (linear // nz) % ny
        x_id = linear // (ny * nz)
        x = origin[0] + (x_id + 0.5) * resolution
        y = origin[1] + (y_id + 0.5) * resolution
        z = origin[2] + (z_id + 0.5) * resolution
        truth_occupied = np.zeros(total, dtype=bool)
        signed_distance = np.full(total, np.inf, dtype=np.float64)
        for center, half in self.boxes:
            q = np.stack((np.abs(x - center[0]) - half[0],
                          np.abs(y - center[1]) - half[1],
                          np.abs(z - center[2]) - half[2]), axis=1)
            outside = np.maximum(q, 0.0)
            sdf = np.linalg.norm(outside, axis=1) + np.minimum(
                np.max(q, axis=1), 0.0)
            signed_distance = np.minimum(signed_distance, sdf)
            truth_occupied |= np.all(q <= 0.0, axis=1)

        known = predicted != 2
        if not np.any(known):
            return None
        pred_occupied = np.zeros(total, dtype=bool)
        pred_free = np.zeros(total, dtype=bool)
        pred_occupied[known] = np.asarray(predicted[known], dtype=np.int8) == 1
        pred_free[known] = np.asarray(predicted[known], dtype=np.int8) == 0
        intersection = int(np.count_nonzero(pred_occupied & truth_occupied))
        union = int(np.count_nonzero(pred_occupied | truth_occupied))
        free_count = int(np.count_nonzero(pred_free))
        free_true = int(np.count_nonzero(pred_free & ~truth_occupied))

        height_metrics = self.evaluate_heightmap(nx, ny, nz, origin,
                                                 resolution)
        if height_metrics is None:
            return None
        distance_valid = np.isfinite(distances)
        estimated_sdf = distances
        errors = estimated_sdf[distance_valid] - signed_distance[distance_valid]
        if errors.size == 0:
            return None
        return {
            'occupied_iou': intersection / union if union else 1.0,
            'occupied_intersection_voxels': intersection,
            'occupied_union_voxels': union,
            'free_space_precision': free_true / free_count if free_count else 0.0,
            'predicted_free_voxels': free_count,
            'true_free_predicted_voxels': free_true,
            'esdf_mae_m': float(np.mean(np.abs(errors))),
            'esdf_rmse_m': float(np.sqrt(np.mean(errors * errors))),
            'esdf_bias_m': float(np.mean(errors)),
            'esdf_sample_count': int(errors.size),
            **height_metrics,
        }

    def evaluate_heightmap(self, nx, ny, nz, origin, resolution):
        message = self.heightmap
        if (message.info.width, message.info.height) != (nx, ny):
            return None
        pred = np.asarray(message.data, dtype=np.int16).reshape(ny, nx)
        encoded = pred.copy()
        pred.fill(-1)
        for value, label in HEIGHT_VALUE.items():
            pred[encoded == value] = label
        z_count = min(nz, int(math.floor(0.50 / resolution)) + 1)
        z_values = origin[2] + (np.arange(z_count) + 0.5) * resolution
        truth = np.zeros((ny, nx), dtype=np.int16)
        truth_known = np.zeros((ny, nx), dtype=bool)
        xs = origin[0] + (np.arange(nx) + 0.5) * resolution
        ys = origin[1] + (np.arange(ny) + 0.5) * resolution
        for center, half in self.boxes:
            inside_xy = ((np.abs(xs[None, :] - center[0]) <= half[0]) &
                         (np.abs(ys[:, None] - center[1]) <= half[1]))
            bottom = max(0, int(math.ceil((center[2] - half[2] - origin[2]) /
                                          resolution - 0.5)))
            top = min(z_count, int(math.ceil((center[2] + half[2] - origin[2]) /
                                             resolution - 0.5)))
            valid_z = z_values[bottom:top]
            valid_z = valid_z[valid_z > 0.02]
            if valid_z.size == 0:
                continue
            lowest = float(np.min(valid_z))
            if lowest <= 0.07:
                label = 3
            elif lowest <= 0.10:
                label = 2
            elif lowest < 0.50:
                label = 1
            else:
                label = 0
            truth[inside_xy] = label
            truth_known |= inside_xy
        # Outside obstacle footprints, the scene is open ground-level space.
        correct = pred == truth
        accuracy = float(np.mean(correct)) if correct.size else 0.0
        known_pred = pred >= 0
        coverage = float(np.mean(known_pred)) if known_pred.size else 0.0
        known_accuracy = (float(np.mean(correct[known_pred]))
                          if np.any(known_pred) else 0.0)
        return {
            'heightmap_accuracy': accuracy,
            'heightmap_known_accuracy': known_accuracy,
            'heightmap_coverage': coverage,
            'heightmap_cells': int(pred.size),
            'heightmap_predicted_cells': int(np.count_nonzero(known_pred)),
            'heightmap_truth_obstacle_cells': int(np.count_nonzero(truth_known)),
        }

    def write_metrics(self, metrics):
        json_path = self.output_dir / 'metrics.json'
        try:
            existing = json.loads(json_path.read_text()) if json_path.exists() else []
        except (json.JSONDecodeError, OSError):
            existing = []
        existing = [row for row in existing
                    if row.get('version') != metrics['version']]
        existing.append(metrics)
        json_path.write_text(json.dumps(existing, indent=2, sort_keys=True) + '\n')
        csv_path = self.output_dir / 'metrics.csv'
        fields = list(metrics.keys())
        write_header = not csv_path.exists() or csv_path.stat().st_size == 0
        with csv_path.open('a', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            if write_header:
                writer.writeheader()
            writer.writerow(metrics)


def main():
    rclpy.init()
    node = MappingEvaluator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
