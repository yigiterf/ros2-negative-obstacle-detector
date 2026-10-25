# Detection Specification

Status: **Draft 1.1** — frozen for simulation bring-up
Owner: Perception
Applies to: `negative_obstacle_detector`, `negative_obstacle_simulation`

This document is the numerical contract for the negative-obstacle detector. Every
threshold in the node configuration must trace back to a row in this document. If a
parameter cannot be traced, it is either unjustified or this document is incomplete.

---

## 1. Purpose

Detect geometric discontinuities that a single-plane 2D lidar cannot observe:

- **negative obstacles**: pits, steps down, loading-dock edges, ramp lips;
- **overhanging obstacles**: table tops, conveyor belts, low pipes, rack shelves;

and project them into the Nav2 costmap as lethal cells, with a bounded false-positive
rate on flat ground.

## 2. System boundary

In scope:

- ingestion and validation of `sensor_msgs/msg/PointCloud2`;
- ground-plane estimation and removal;
- negative-obstacle extraction;
- overhang extraction within the robot's vertical envelope;
- projection to a 2D lethal-cell representation;
- diagnostics, latency and memory instrumentation.

Out of scope for v1:

- SLAM, localization, sensor fusion beyond point-cloud merging;
- dynamic-object tracking;
- semantic classification (person / pallet / forklift);
- changes to Nav2 planners or controllers;
- hardware drivers.

## 3. Conventions

- Units: SI (m, s, rad). Angles in configuration files are radians unless the
  parameter name ends in `_deg`.
- Body frame: REP-103 (`x` forward, `y` left, `z` up).
- Ground frame: REP-105 (`map -> odom -> base_footprint -> base_link`).
- Optical frame: REP-103 camera convention (`z` forward, `x` right, `y` down).
- The detector never assumes the input cloud is expressed in `base_link`. The source
  frame is read from `PointCloud2.header.frame_id` and transformed explicitly.
- Pitch sign: URDF and Gazebo use the right-hand rule, where a positive rotation
  about `+y` tips the `+x` (forward) axis toward `-z` (down). Sensor pitch in this
  document is reported as tilt below the horizontal; the equivalent URDF rpy pitch
  is positive (13.5 deg downward = rpy pitch +0.2356 rad).
- All nodes run with `use_sim_time:=true` in simulation.

## 4. Reference platform

| Property | Value |
|---|---|
| Drive type | differential |
| Footprint | 0.60 m (width) x 0.90 m (length) |
| Body height | 1.20 m |
| Maximum linear speed | 0.8 m/s |
| Ground clearance | 0.08 m |
| Sensor head height | 0.45 m (RGB-D), 0.60 m (3D lidar) |

The footprint and height are safety-critical parameters. The costmap projection uses
`robot_height` to decide whether a 3D return is a collision or a pass-under.

## 5. Sensor suite

### 5.1 RGB-D camera (primary, near field)

| Property | Value |
|---|---|
| Mount height | 0.45 m above ground |
| Pitch (below horizontal) | 13.5 deg |
| Horizontal FoV | 87 deg |
| Vertical FoV | 58 deg |
| Resolution | 640 x 480 |
| Frame rate | 15 Hz |
| Depth range | 0.4 - 6.0 m |
| Depth noise (simulated) | sigma = 0.5 cm + 0.02 * z^2 / 1 m |

Rationale for the pitch: the upper ray reaches +15.5 deg, sufficient to see the
underside of a 0.71 m surface at 1.5-3.0 m. The lower ray reaches -42.5 deg, covering
the ground from 0.49 m in front of the sensor.

### 5.2 3D lidar (secondary, 360 deg)

| Property | Value |
|---|---|
| Mount height | 0.60 m above ground |
| Pitch (below horizontal) | 5 deg |
| Horizontal FoV | 360 deg |
| Vertical FoV | 40 deg (-25 deg to +15 deg relative to axis) |
| Channels | 32 |
| Frame rate | 10 Hz |
| Range | 0.5 - 30 m |
| Range noise | sigma = 2 cm |

Rationale: the lidar provides angular coverage and low range noise, but its vertical
sampling is sparse. At 2.5 m the ground sample spacing is approximately 10 cm, which
is marginal for a 0.6 m pit. The RGB-D remains the primary negative-obstacle sensor.

## 6. Detection targets (ground truth)

| ID | Feature | Geometry | Pose (x, y) | Primary cue |
|---|---|---|---|---|
| N1 | Narrow pit | 0.60 x 0.60 m, 0.20 m deep, vertical walls | (2.50, 0.00) | missing ground + far-wall points 10.8 cm below plane at 2.5 m |
| N2 | Wide shallow pit / step | 1.20 x 1.20 m, 0.15 m deep, vertical walls | (-4.00, 4.50) | far-wall points + missing ground |
| N3 | Loading-dock edge | 0.30 m drop, 3.0 m edge length | x = 7.50, y in [-7.50, -4.50] | missing ground beyond edge |
| N4 | Loading ramp | 10 deg incline, 2.0 m wide, 0.30 m rise | x in [5.80, 7.50], y in [-7.00, -5.00] | slope must not be classified as negative |

## 7. Overhang and positive controls

| ID | Feature | Geometry | Pose | Purpose |
|---|---|---|---|---|
| O1 | Workbench top | 1.20 x 0.80 m, top at 0.75 m, 5 cm thick | (2.00, 3.20) | overhang at robot height |
| O2 | Conveyor belt | 4.00 x 0.70 m, top at 0.90 m | (-8.00, 2.80) | overhang, long edge |
| O3 | Low pipe | diameter 0.08 m, centre at 1.10 m | (4.00, -4.00), along y | low overhang |
| P1 | Rack shelves | 2.40 x 1.00 x 3.00 m | (+-6.00 / 0.00, +-6.00) | shelves at 0.5 / 1.5 / 2.5 m |
| P2 | Pallets | 1.20 x 0.80 x 0.14 m + boxes | (-8.50, -6.50), (5.50, 6.50), (10.50, -6.50) | positive obstacles |
| P3 | Control boxes | 0.30 - 0.60 m cubes | see world | false-positive control |
| P4 | Bollards | diameter 0.16 m, height 0.80 m | see world | false-positive control |

## 8. Acceptance criteria

Measured on the simulation world in Section 9, RGB-D configuration, 10 Hz:

| Metric | Target | Method |
|---|---|---|
| Recall, N1 at <= 2.5 m | >= 0.90 | per-frame detection, 100 frames |
| Recall, N2 at <= 3.0 m | >= 0.90 | per-frame detection, 100 frames |
| False-positive rate, flat ground | <= 0.02 | frames with any lethal cell outside ground-truth masks |
| Edge localization error, N3 | <= 0.15 m | distance from projected edge to ground-truth edge |
| End-to-end latency | p95 <= 50 ms | timestamp at callback entry to publish |
| Memory | no monotonic growth over 10 min | RSS sampled at 1 Hz |
| Numerical safety | no NaN/Inf in output | assert in debug builds |

The lidar configuration is measured separately and reported, not gated, in v1.

## 9. Simulation world

World file: `negative_obstacle_simulation/worlds/negative_obstacle_test.sdf`.

Factory envelope: 24 x 18 m, walls at x = +-12 m and y = +-9 m, open roof with beams
at 5.5 m. Main aisle along x, y in [-2, 2]. Robot start pose: (0, 0, 0), yaw 0.

Key coordinates:

| Feature | Pose / extent |
|---|---|
| Pit N1 | centre (2.50, 0.00), opening 0.60 x 0.60 m, floor at -0.20 m |
| Pit N2 | centre (-4.00, 4.50), opening 1.20 x 1.20 m, floor at -0.15 m |
| Dock N3 | platform x in [7.50, 11.50], y in [-7.50, -4.50], top at +0.30 m |
| Ramp N4 | x in [5.80, 7.50], y in [-7.00, -5.00], 10 deg |
| Workbench O1 | centre (2.00, 3.20), top at 0.75 m |
| Conveyor O2 | centre (-8.00, 2.80), top at 0.90 m |
| Pipe O3 | centre (4.00, -4.00), z = 1.10 m, length 3.80 m along y |

The world contains no dynamic actors. This is deliberate: the first regression suite
must isolate perception error from scene motion.

## 10. Parameter traceability

| Parameter | Value | Derived from |
|---|---|---|
| `sensor.min_range` | 0.40 m | RGB-D spec, Section 5.1 |
| `sensor.max_range` | 6.00 m | RGB-D spec, Section 5.1 |
| `ground.ransac_distance_threshold` | 0.03 m | 3 sigma of depth noise at 2.5 m |
| `ground.radius_clip` | 0.50 m | near-field ground coverage |
| `negative.min_depth` | 0.10 m | N1/N2 with 50% margin |
| `negative.max_detection_range` | 2.50 m | ground sample spacing <= 3 cm |
| `negative.min_cluster_points` | 8 | angular resolution vs. 0.6 m pit |
| `overhang.min_height` | 0.35 m | sensor height minus mounting tolerance |
| `overhang.max_height` | 1.20 m | robot height |
| `projection.cell_size` | 0.05 m | Nav2 costmap resolution |
| `projection.inflation_radius` | 0.30 m | half robot width |

The values are initial estimates. They are frozen only after the measurement campaign
in Section 8; any change requires a note in the pull request and a re-run of the
regression bag.

## 11. Known limitations

- RGB-D depth noise grows quadratically with range. A 0.20 m pit at 3.5 m is close to
  the noise floor; v1 restricts negative-obstacle claims to 2.5 m.
- Transparent, specular and textureless surfaces are not modelled in v1.
- The 32-beam lidar undersamples small pits; it is not the primary negative-obstacle
  channel.
- The world has no dynamic actors, wind, or sensor dropout. Those are added after the
  static regression suite is green.
- Ground truth is geometric, not semantic. A detected pit is a set of cells, not an
  object with an identity.
