# ros2-negative-obstacle-detector

Detects **negative obstacles** (pits, steps, loading-dock edges) and **overhanging
obstacles** (table tops, low pipes) that a single-plane 2D lidar cannot see, and
reports them to the Nav2 costmap as lethal cells.

The detector consumes 3D point clouds (RGB-D and 3D lidar), estimates the ground
plane with RANSAC, extracts geometric discontinuities, and projects them into a 2D
costmap-friendly representation via a custom Nav2 costmap layer.

> **Status:** all four phases complete and verified end-to-end in simulation —
> a 0.6 x 0.6 m, 20 cm deep pit is marked lethal at 2.5 m range.

## Motivation

A 2D lidar scans a single horizontal plane. A 20 cm pit is below that plane; from
the lidar's perspective it is indistinguishable from out-of-range space. The same
applies to a table top above the scan plane. This repository adds the missing
third dimension.

## Pipeline

```
PointCloud2 -> TF (base_link) -> finite filter -> PassThrough -> VoxelGrid
           -> RANSAC ground plane -> signed-distance classification
           -> Euclidean clustering -> hazard cloud -> Nav2 costmap layer
           -> LETHAL cells
```

## Packages

| Package | Responsibility |
|---|---|
| `negative_obstacle_description` | URDF/Xacro model of the reference AMR |
| `negative_obstacle_simulation` | Gazebo world, bridge config, launch files |
| `negative_obstacle_detector` | Perception node (pre-processing, RANSAC segmentation, obstacle extraction) |
| `negative_obstacle_costmap_plugin` | Nav2 costmap layer that marks hazards lethal |
| `negative_obstacle_bringup` | Full-stack launch, Nav2 config, RViz config |

## Requirements

- Ubuntu 24.04, ROS 2 Jazzy
- Gazebo Harmonic (via `ros_gz_sim`), PCL 1.14 (`libpcl-dev`), Nav2

## Build

```bash
source /opt/ros/jazzy/setup.bash
# clone into the src/ directory of a colcon workspace
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
# Full stack: simulation, detector, Nav2 costmap, RViz
ros2 launch negative_obstacle_bringup negative_obstacle.launch.py

# Headless (no Gazebo GUI)
ros2 launch negative_obstacle_bringup negative_obstacle.launch.py headless:=true
```

RViz shows the costmap (`/costmap`), the negative-obstacle points
(`/negative_obstacle/negative`) and markers (`/negative_obstacle/markers`).

## Documentation

- `docs/detection-spec.md` — numerical contract: geometry, targets, acceptance criteria.
- The `/learning` notes and the engineering whitepaper are intentionally not
  versioned (personal study material).

## License

Apache-2.0
