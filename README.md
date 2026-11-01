# Sapphire ROS 1

ROS 1 Noetic interface for the ROS-independent `sapphire` LiDAR-inertial
odometry core.

This directory contains a native ROS 1 Noetic catkin package backed by the
shared ROS-independent `sapphire` core. It consumes generic PointCloud2 and
IMU messages and has no sensor-driver dependency.

## Documents

- [ROS 1/ROS 2 interface contract](docs/INTERFACE_CONTRACT.md)
- [HILTI 2023 reproduction after reboot](docs/HILTI_REPRODUCTION.md)
- [Migration design](docs/MIGRATION.md)
- [Docker toolchain](docs/DOCKER.md)
- [HILTI 2023 drift tuning](docs/HILTI_TUNING.md)

Start with `docs/HILTI_REPRODUCTION.md` when resuming after a host reboot. It
contains the exact container, build, replay, RViz, evaluation, artifact, and
recovery commands.

## Planned package layout

```text
sapphire_ros1/
├── CMakeLists.txt
├── package.xml
├── include/sapphire_ros1/
├── launch/
├── src/
├── test/
└── docker/
```

The core remains a separate sibling package:

```text
src/
├── sapphire/
├── sapphire_ros1/
└── sapphire_ros2/
```

ROS 1 and ROS 2 are sibling interface layers. Neither interface is allowed to
become a dependency of the core.

## Build and run

Inside the pinned Noetic image:

```bash
cd /workspace
catkin config --extend /opt/ros/noetic \
  --build-space /workspace/build_ros1 \
  --devel-space /workspace/devel_ros1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build sapphire_ros1
source devel_ros1/setup.bash
```

From the host, start the ROS 1 development container with the HILTI dataset
mounted read-only at `/data`:

```bash
docker compose -f docker-compose.ros1.yml up -d
docker exec -it sapphire_ros1_dev bash
```

The existing `docker-compose.yml` remains the ROS 2 development service.

Start the node without RViz:

```bash
roslaunch sapphire_ros1 sapphire.launch \
  config_file:=/workspace/src/sapphire/cfg/sapphire_mid360.toml
```

Override input and output topics or enable RViz with launch arguments:

```bash
roslaunch sapphire_ros1 sapphire.launch \
  config_file:=/workspace/src/sapphire/cfg/sapphire_mid360.toml \
  lidar_topic:=/velodyne_points \
  imu_topic:=/imu_raw \
  odom_topic:=/odometry \
  rviz:=true
```

For HILTI interactive checks, use the lightweight RViz profile:

```bash
roslaunch sapphire_ros1 hilti_2023_handheld.launch \
  config_file:=/workspace/src/sapphire/cfg/sapphire_hilti_2023_handheld_tuned.toml \
  bag:=/data/rosbags/site1_handheld_5.bag \
  rviz:=true \
  rviz_config:=/workspace/src/sapphire_ros1/launch/visual_check.rviz \
  trajectory_output:=/workspace/results/hilti_2023/reproduction/trajectory.tum
```

HILTI SLAM Challenge 2023 profiles use the supplied calibration and duplicate
the LiDAR clock offset into every point timestamp:

```bash
# Handheld: PandarXT-32 + Alphasense IMU
roslaunch sapphire_ros1 hilti_2023_handheld.launch \
  config_file:=/workspace/src/sapphire/cfg/sapphire_hilti_2023_handheld_tuned.toml \
  bag:=/data/rosbags/site1_handheld_1.bag

# Robot: RoboSense BPearl + Xsens MTi-600 IMU
roslaunch sapphire_ros1 hilti_2023_robot.launch \
  config_file:=/workspace/src/sapphire/cfg/sapphire_hilti_2023_robot.toml \
  bag:=/data/rosbags/site2_robot_1.bag
```

Both HILTI launches publish odometry, TF, and TUM trajectories in the IMU
frame, matching the challenge submission convention.

For a bag that publishes `/clock`, add `use_sim_time:=true`. The node uses two
spinner threads so IMU callbacks remain live while LiDAR processing waits for
scan-end IMU coverage.

## Tests

```bash
catkin build sapphire_ros1 --no-deps --make-args tests
catkin test sapphire_ros1
```

The conversion tests cover nanosecond timestamps, IMU scaling fields, custom
point cloud serialization/ingestion, frames, and odometry velocity conversion.
Bag replay, TF connectivity, callback progress, and ROS 1/ROS 2 parity remain
runtime tests for the pinned image.

The pinned image supplies Eigen, PCL, GTSAM, small_gicp, toml++, and spdlog
from `/opt/sapphire-deps`. These source-pinned algorithm dependencies are not
resolved by `rosdep`; use the image rather than installing a second system PCL.

## Status

The catkin wrapper, launch/RViz configuration, conversion tests, and in-node
TUM trajectory export are implemented. The adapter builds in the pinned
Noetic image and its conversion tests pass. HILTI bag replay, RViz
visualization, trajectory persistence, and SE(3)-aligned sparse-reference
evaluation have been exercised on `site1_handheld_5` and
`site1_handheld_2`. See `docs/HILTI_TUNING.md` for measured results and
`docs/HILTI_REPRODUCTION.md` for the restart procedure.

