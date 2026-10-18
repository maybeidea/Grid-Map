# Dual-resolution plan environment

## Purpose

The plan environment maintains two independent voxel maps for one task:

- **Original/high-resolution map**: the task polygon bounding box, normally
  0.01 m voxels. Occupancy, optional TSDF, and ESDF are accumulated through
  the stationary-frame gate and committed only after the configured quota.
  This is the source for the height map.
- **Expanded/low-resolution map**: the expanded task rectangle, normally
  0.03 m voxels. Occupancy, optional TSDF, and ESDF are updated for every
  accepted point cloud, including clouds received while the robot is moving.
  It supplies the realtime safety context outside the original map.

The two maps have independent geometry, Occupancy log-odds models, ESDF
policies, and TSDF enable/truncation settings. They share only the filtered
sensor cloud and TF pose.

## Query policy

The existing GridMap::getDistance(), getPreciseDistance(), and
GridMap::getOccupancy() interfaces remain unchanged. A point is queried from
the high-resolution snapshot when its voxel lies in the original map geometry.
Otherwise it is queried from the expanded low-resolution geometry. Points
outside both maps return the legacy out-of-map values (100000.0 for distance
and -1 for occupancy).

EsdfSnapshot::geometry and the original occupancy/ESDF arrays are the
high-resolution compatibility fields. The expanded map is exposed through
expanded_geometry and the corresponding expanded_* arrays.

## Task lifecycle

1. startTaskMapping() computes the original polygon bounding box and the
   expanded rectangle, then allocates both map volumes.
2. Each valid cloud updates the expanded map immediately and recomputes its
   ESDF.
3. The same cloud is admitted to the high-resolution local accumulator only
   when the motion gate reports a stationary pose.
4. Once the stationary quota is reached, the local high-resolution map is
   registered and committed to the global high-resolution map; its ESDF and a
   new immutable snapshot are published.
5. stopTaskMapping() invalidates both maps and queued clouds.

The expanded map is deliberately not blocked by the stationary gate. This
keeps obstacle queries current while the robot is moving, while the
high-resolution map remains stable enough for height-map generation.

## Configuration

Existing grid_map.resolution remains the high-resolution alias. New
parameters are:

    grid_map:
      high_resolution: 0.01
      low_resolution: 0.03
      low_p_hit: 0.70
      low_p_miss: 0.35
      low_p_min: 0.12
      low_p_max: 0.97
      low_p_occ: 0.80
      low_esdf_unknown_mode: task_roi
      low_esdf_roi_z_max: 0.50
      low_esdf_signed_distance: true
      low_esdf_max_distance: 100000.0
      low_tsdf:
        enabled: false
        truncation_distance: 0.05

grid_map.resolution and grid_map.high_resolution are aliases for the
original map; if both are supplied, the legacy resolution spelling wins.
The old configure() API still configures the original map and uses 0.03 m as
the expanded-map default. New code can call configureDualResolution().

## Real-time considerations

The low-resolution ESDF is recomputed after each accepted cloud. Its voxel
count is intentionally lower than the original map's count. ESDF's three
axis distance transforms are parallelized with OpenMP; each line owns its
temporary buffers, so the transform is thread-safe. The point-cloud mailbox
still retains only the newest cloud, preventing unbounded queue latency.

The high-resolution ESDF and ICP remain on the stationary commit path. Both
maps currently share one mapping worker, so a high-resolution commit can still
create a bounded pause in low-resolution refreshes; the mailbox drops stale
clouds instead of allowing that pause to become an unbounded queue. Production
tuning should measure filtered point count, ray traversal time,
low-resolution ESDF time, high-resolution commit time, and dropped mailbox
frames. A deployment that requires a strict low-layer deadline should move
high-resolution commit to a second worker with an explicit job queue.
