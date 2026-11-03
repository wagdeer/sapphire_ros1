# Sapphire ROS 1

ROS 1 Noetic interface for [Sapphire](https://github.com/wagdeer/sapphire).

Put the core and this package in the same catkin workspace. The core is found
as the sibling directory `../sapphire`.

```text
catkin_ws/src/
├── sapphire/          https://github.com/wagdeer/sapphire
└── sapphire_ros1/
```

Further reading:

- [docs/DOCKER.md](docs/DOCKER.md): pinned toolchain and X11
- [docs/INTERFACE_CONTRACT.md](docs/INTERFACE_CONTRACT.md): topics, frames, and queues
- [docs/HILTI_REPRODUCTION.md](docs/HILTI_REPRODUCTION.md): HILTI 2023 handheld replay
- [sapphire/docs/DATASET_REPLAY.md](https://github.com/wagdeer/sapphire/blob/master/docs/DATASET_REPLAY.md): dataset replay in the core repository

## Docker

From this repository:

```bash
docker build -f docker/Dockerfile -t sapphire:noetic .
```

The image is Ubuntu 20.04 with ROS Noetic, and already contains Eigen 3.4.0, PCL 1.14.1, GTSAM 4.3.0, small_gicp 1.0.1, spdlog, and toml++ 3.4.0.

Mount a workspace that already contains `src/sapphire` and `src/sapphire_ros1`:

```bash
docker run --rm -it -v ~/catkin_ws:/workspace sapphire:noetic
cd /workspace
catkin_make
source devel/setup.bash
```

The image entrypoint sources `/opt/ros/noetic` and, when present,
`devel_ros1/setup.bash` or `devel/setup.bash`. A workspace that also contains
a ROS 2 build should use the separate `devel_ros1` layout in
[docs/DOCKER.md](docs/DOCKER.md) instead of `devel/`. RViz needs the X11
forwarding shown there. `foxglove_bridge` is not in this image; it is added by
`docker/Dockerfile.foxglove`.

## Without Docker

Install Noetic and the packages that come from apt:

```bash
sudo apt install build-essential cmake git pkg-config \
  libboost-all-dev libflann-dev libomp-dev libspdlog-dev \
  ros-noetic-pcl-conversions ros-noetic-rosbag \
  ros-noetic-rviz ros-noetic-tf2-ros
```

Noetic's Eigen and PCL are too old. Build these from source, Eigen first, and install them where CMake can find them:

| Library | Version |
|---|---|
| [Eigen](https://gitlab.com/libeigen/eigen) | 3.4.0 |
| [PCL](https://github.com/PointCloudLibrary/pcl) | 1.14.1 |
| [GTSAM](https://github.com/borglab/gtsam) | 4.3.0 |
| [small_gicp](https://github.com/koide3/small_gicp) | 1.0.1 |
| [toml++](https://github.com/marzer/tomlplusplus) | 3.4.0 |

On Ubuntu 20.04, configure GTSAM with `-DGTSAM_WITH_TBB=OFF` and apply `docker/patches/gtsam-4.3.0-empty-list.patch` before building. The Dockerfile is the reference for the other configure flags. In particular it builds PCL with VTK turned off, which avoids an extra MPI dependency. Install Eigen before PCL, GTSAM, and small_gicp, and point those builds at that Eigen.

Then build the workspace:

```bash
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Run

`config_file` is required. The topic arguments below override the launch
defaults `/front_lidar` and `/front_lidar/imu`.

```bash
roslaunch sapphire_ros1 sapphire.launch \
  config_file:=/path/to/sapphire/cfg/sapphire_mid360.toml \
  lidar_topic:=/your_lidar \
  imu_topic:=/your_imu \
  rviz:=true
```

The node always publishes `/sapphire/odometry` and `/sapphire/deskewed`.
`/sapphire/trajectory`, `/sapphire/pgo/map`, and `/sapphire/pgo/graph` are
published only when the profile sets `pgo.enabled` to true.

To play a bag, add `use_sim_time:=true` to the launch command, then in another terminal:

```bash
rosbag play --clock your.bag
```

Stop the launch with Ctrl-C so the TUM file can be written. A killed process
does not flush it. `trajectory_output:=/path/to/trajectory.tum` overrides the
path in the TOML profile.

HILTI 2023 handheld recordings can be started directly:

```bash
roslaunch sapphire_ros1 hilti_2023_handheld.launch \
  config_file:=/path/to/sapphire/cfg/sapphire_hilti_2023_handheld.toml \
  bag:=/path/to/site1_handheld_1.bag \
  rviz:=true
```
