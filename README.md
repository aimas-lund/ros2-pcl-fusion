# ros2-pcl-fusion

[![Build Docker Image](https://img.shields.io/github/actions/workflow/status/aimas-lund/ros2-pcl-fusion/build.yaml?branch=main&label=Docker%20Image)](https://github.com/aimas-lund/ros2-pcl-fusion/actions/workflows/on-push.yaml)
[![License](https://img.shields.io/github/license/aimas-lund/ros2-pcl-fusion)](https://github.com/aimas-lund/ros2-pcl-fusion/blob/main/LICENSE)
[![Last commit](https://img.shields.io/github/last-commit/aimas-lund/ros2-pcl-fusion)](https://github.com/aimas-lund/ros2-pcl-fusion/commits/main)
[![Issues](https://img.shields.io/github/issues/aimas-lund/ros2-pcl-fusion)](https://github.com/aimas-lund/ros2-pcl-fusion/issues)
[![Stars](https://img.shields.io/github/stars/aimas-lund/ros2-pcl-fusion?style=social)](https://github.com/aimas-lund/ros2-pcl-fusion/stargazers)

## TODO

- [ ] Option to only perform cloud validation on first synced message
- [ ] Benchmark scripts
- [ ] Define performance goals
- [ ] Flexible PCL fusion (pcl_conversion)

## Overview

`ros2-pcl-fusion` provides a small ROS 2 node that fuses two incoming `sensor_msgs/msg/PointCloud2` streams at runtime.

Fusion in this package is performed in the following steps:

- each input cloud is transformed into a common output frame via TF2
- the clouds are fused by concatenating the raw `PointCloud2.data` buffers

## The node is designed to

### Build the image (Jazzy)

- Subscribe to two `PointCloud2` topics.
- Time-align the two streams using `message_filters::ApproximateEpsilonTime`.
- Transform both clouds into `output.frame_id` using TF2.
- Publish a single fused `PointCloud2` on `output.topic`.
- Stamp the output message using the newer of the two transformed cloud stamps (note: transformation uses the TF stamp).
- Produce an unorganized output cloud.

## The node does NOT do (but might in the future)

- No point cloud registration/alignment (no ICP / NDT / scan matching).
  - The node assumes the TF tree already provides the correct relative transforms.
- No filtering, downsampling, de-duplication, outlier removal, etc.
- No field conversion or “best effort” merging when point layouts differ.
  - If the two `PointCloud2` layouts differ, fusion is skipped.
- No support for fusing more than two point clouds.

## Point cloud matching requirements

Fusion uses a fast “byte concatenation” approach, so the two input clouds must be compatible.

### 1) Frames / transforms must be available

- Each incoming message's `header.frame_id` must be transformable to `output.frame_id`.
- If a transform is missing (or TF lookup fails), fusion is skipped.

### 2) `PointCloud2` layout must match exactly

After both clouds are in the output frame, their layouts must match:

- `point_step` must match
- `is_bigendian` must match
- `fields` must match element-by-element (`name`, `offset`, `datatype`, `count`)
- `data.size()` must be divisible by `point_step` for both messages

If any of these checks fail, the node does not publish.

### 3) Output cloud is unorganized

This node **does not preserve** any organized point cloud structure.

- The published cloud always uses `height = 1`.
- Any `height/width` organization (e.g., image-like LiDAR rings) is not retained.

## Topics

- Inputs (2 topics): `input.topics[0]`, `input.topics[1]`
- Output (1 topic): `output.topic`

## Parameters

The node declares parameters on startup and supports overrides.

- `input.topics` (string[2]): two `PointCloud2` input topics
- `input.frame_ids` (string[2]): expected frames for the inputs (currently used for logging only; TF uses `header.frame_id`)
- `input.sync.queue_size` (int, default `10`): ApproximateEpsilonTime queue size
- `input.sync.epsilon_ms` (int, default `100`): ApproximateEpsilonTime max interval duration, in milliseconds (e.g. `100` = 0.1s)
- `output.topic` (string, default `/fused_pointcloud`): output topic
- `output.frame_id` (string, default `fused_frame`): output frame
- `transform.type` (string, `static|dynamic`, default `static`):
  - `static`: caches the TF lookup per source frame
  - `dynamic`: looks up TF each callback

An example parameter file lives at: `src/config/params.yaml`.

## Build

### Supported ROS distros

This repo currently supports ROS2 `humble` and `jazzy`. The build automatically checks `ROS_DISTRO` to select the right compile-time settings.


From the repo root:

```bash
colcon build
source install/setup.bash
```

## Run

### Run directly with the parameter file

```bash
source install/setup.bash
ros2 run fusion fusion_node_exe --ros-args --params-file src/config/params.yaml
```

### Run with launch file

The package includes a launch file at `src/launch/fusion.launch.py`:

```bash
source install/setup.bash
ros2 launch fusion fusion.launch.py
```

## Docker

This repo builds a container that runs `ros2 launch fusion fusion.launch.py` by default.

### Build the image (Jazzy)

```bash
docker build -t pcl-fusion:jazzy --build-arg ROS_DISTRO=jazzy .
```

### Build the image (Humble)

```bash
docker build -t pcl-fusion:humble --build-arg ROS_DISTRO=humble .
```

### Ports
If you don't want to attach the containerized fusion node on the hosts machine's network with `--network=host`, you can use [this article](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Domain-ID.html) to figure out what ports to expose.
