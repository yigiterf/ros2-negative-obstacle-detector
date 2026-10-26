# ros2-negative-obstacle-detector

Detects **negative obstacles** (pits, steps, loading-dock edges) and **overhanging
obstacles** (table tops, low pipes) that a single-plane 2D lidar cannot see, and
reports them to the Nav2 costmap as lethal cells.

The detector consumes 3D point clouds (RGB-D and 3D lidar), estimates the ground
plane, extracts geometric discontinuities and projects them into a 2D
costmap-friendly representation. It targets differential-drive AMRs operating in
warehouse and factory environments.

> **Status:** Phase 2 (perception pipeline) complete and verified in simulation.
> Ground segmentation (RANSAC) and costmap projection are in progress.

## Motivation

A 2D lidar scans a single horizontal plane. A 20 cm pit is below that plane; from
the lidar's perspective it is indistinguishable from out-of-range space. The same
applies to a table top above the scan plane. This repository adds the missing
third dimension.

## Features

- Gazebo Harmonic factory world with ground-truth negative obstacles, overhangs,
  storage racks, loading dock and conveyor (see `docs/detection-spec.md`).
- Reference AMR description (URDF/Xacro) with RGB-D camera, 3D lidar, 2D lidar and
  IMU, following REP-103/105.
- ROS 2 `ros_gz_bridge` topic mapping and launch files.
- `negative_obstacle_detector` LifecycleNode with a reusable, allocation-free
  pre-processing stage (finite filter, PassThrough, VoxelGrid).

## Packages

| Package | Responsibility |
|---|---|
| `negative_obstacle_description` | URDF/Xacro model of the reference AMR |
| `negative_obstacle_simulation` | Gazebo world, bridge config, launch files |
| `negative_obstacle_detector` | Perception node (pre-processing, segmentation, projection) |
| `negative_obstacle_costmap_plugin` | Nav2 costmap layer (planned) |

## Requirements

- Ubuntu 24.04, ROS 2 Jazzy
- Gazebo Harmonic (via `ros_gz_sim`)
- PCL 1.14 (`libpcl-dev`), Nav2

## Build

```bash
source /opt/ros/jazzy/setup.bash
# clone into the src/ directory of a colcon workspace
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
# Simulation with GUI (factory world + robot + sensors)
ros2 launch negative_obstacle_simulation simulation.launch.py

# Headless
ros2 launch negative_obstacle_simulation simulation.launch.py headless:=true
```

Then inspect the sensor data:

```bash
ros2 topic list
ros2 topic hz /depth_camera/points
```

## Documentation

- `docs/detection-spec.md` — numerical contract: geometry, targets, acceptance criteria.
- The `/learning` notes are intentionally not versioned (personal study material).

## License

Apache-2.0
