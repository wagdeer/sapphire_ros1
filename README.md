# Sapphire ROS 1

ROS 1 Noetic interface for [Sapphire](https://github.com/wagdeer/sapphire).

Put this package next to the core. The build uses `../sapphire` directly.

```text
catkin_ws/src/
├── sapphire/
└── sapphire_ros1/
```

[sapphire_ros2](https://github.com/wagdeer/sapphire_ros2) is the ROS 2 Humble package. Topic names, frames, and queue sizes are the same and are listed in [docs/INTERFACE_CONTRACT.md](docs/INTERFACE_CONTRACT.md).

## Docker

Use the image. It already has Noetic and the libraries.

```bash
docker build -f docker/Dockerfile -t sapphire:noetic .
docker run --rm -it -v ~/catkin_ws:/workspace sapphire:noetic
```

`~/catkin_ws/src` needs both `sapphire` and `sapphire_ros1`. Inside the container:

```bash
cd /workspace
catkin_make
source devel/setup.bash
```

X11 and a separate `devel_ros1` space are in [docs/DOCKER.md](docs/DOCKER.md).

## Without Docker

| Library | Version | Ubuntu 20.04 |
|---|---|---|
| Eigen | 3.4.0 | too old in apt; build it |
| PCL | 1.14.1 | too old in apt; build it |
| OpenMP | | `libomp-dev` |
| spdlog | | `libspdlog-dev` |
| GTSAM | 4.3.0 | built against Eigen 3.4 in the Dockerfile |
| small_gicp | 1.0.1 | build `v1.0.1` |

This package's CMake requires PCL 1.14, so the apt PCL 1.10 package cannot be used. toml++ 3.4 is downloaded by CMake when it is not already installed. Message and TF packages are listed in `package.xml`.

```bash
sudo apt install build-essential cmake git libomp-dev libspdlog-dev
```

Eigen 3.4.0, PCL 1.14.1, GTSAM 4.3.0, and small_gicp 1.0.1 are built in [docker/Dockerfile](docker/Dockerfile). GTSAM is tag `4.3.0` with `GTSAM_WITH_TBB=OFF` and a small compatibility patch for empty CMake glob lists. This package needs Eigen 3.4, so it uses the Dockerfile build.

```bash
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Run

`config_file` is required. The topic arguments override `/front_lidar` and `/front_lidar/imu`.

```bash
roslaunch sapphire_ros1 sapphire.launch \
  config_file:=/path/to/sapphire/cfg/mid360.toml \
  lidar_topic:=/your_lidar \
  imu_topic:=/your_imu \
  rviz:=true
```

The node always publishes `/sapphire/odometry` and `/sapphire/deskewed`. It publishes `/sapphire/trajectory`, `/sapphire/pgo/map`, and `/sapphire/pgo/graph` when the profile sets `pgo.enabled` to true.

## Play a bag

```bash
roslaunch sapphire_ros1 sapphire.launch \
  config_file:=/path/to/sapphire/cfg/mid360.toml \
  lidar_topic:=/your_lidar \
  imu_topic:=/your_imu \
  use_sim_time:=true \
  trajectory_output:=/path/to/trajectory.tum
```

In another terminal:

```bash
rosbag play --clock your.bag
```

Stop the launch with Ctrl-C after the bag ends. That writes `trajectory.tum`. `kill -9` does not. `trajectory_output` replaces the path stored in the TOML profile. Pass it: several profiles point at `/workspace/results/...`.

Dataset profiles, HILTI launch files, and bag conversion are in [DATASET_REPLAY.md](https://github.com/wagdeer/sapphire/blob/master/docs/DATASET_REPLAY.md).

A HILTI handheld bag can be started in one command. The launch file sets the topics and frames. `config_file` is still required.

```bash
roslaunch sapphire_ros1 hilti_2023_handheld.launch \
  config_file:=/path/to/sapphire/cfg/hilti_2023_handheld.toml \
  bag:=/path/to/site1_handheld_1.bag \
  rviz:=true
```

`hilti_2022_handheld.launch` and `hilti_2023_robot.launch` work the same way with `cfg/hilti_2022_handheld.toml` and `cfg/hilti_2023_robot.toml`. The three commands are in [docs/HILTI_REPRODUCTION.md](docs/HILTI_REPRODUCTION.md).

## License

MIT.
