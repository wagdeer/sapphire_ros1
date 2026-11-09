# Docker

The `sapphire:noetic` image is a build environment: Ubuntu 20.04, ROS Noetic, and the libraries below. Source stays in the mounted workspace.

| Component | Version |
|---|---|
| Ubuntu / ROS | 20.04 / Noetic |
| Eigen | 3.4.0 |
| PCL | 1.14.1 |
| GTSAM | 4.2.0 |
| small_gicp | 1.0.1 |
| toml++ | 3.4.0 |

These libraries are installed under `/opt/sapphire-deps`, so CMake does not pick up Ubuntu's Eigen 3.3 or PCL 1.10. GTSAM 4.2.0 is built against that Eigen 3.4 with `GTSAM_WITH_TBB=OFF`. The Ubuntu 20.04 package from `ppa:borglab/gtsam-release-4.2` is built against Eigen 3.3.

## Build the image

From this repository, matching your user id so files written in the container stay editable on the host:

```bash
docker build \
  -f docker/Dockerfile \
  --build-arg USER_UID="$(id -u)" \
  --build-arg USER_GID="$(id -g)" \
  -t sapphire:noetic \
  .
```

The default user id is 1000. The default apt mirrors are Tsinghua. Override them with `UBUNTU_MIRROR` and `ROS_MIRROR` when the official archives are faster.

## Run

Mount the whole workspace, including `src/sapphire`:

```bash
docker run --rm -it \
  -v ~/catkin_ws:/workspace \
  sapphire:noetic
```

The entrypoint sources `/opt/ros/noetic` and, when present, `devel_ros1/setup.bash` or `devel/setup.bash`.

## Build the workspace

ROS 1 only:

```bash
cd /workspace
catkin_make
source devel/setup.bash
```

The same tree also used for ROS 2 should keep a separate devel space:

```bash
cd /workspace
catkin config \
  --extend /opt/ros/noetic \
  --build-space /workspace/build_ros1 \
  --devel-space /workspace/devel_ros1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build sapphire_ros1
source devel_ros1/setup.bash
```

The core is the plain CMake project at `src/sapphire`. This package adds it with `add_subdirectory`.

## RViz

```bash
xhost +local:docker
docker run --rm -it \
  -e DISPLAY="$DISPLAY" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v ~/catkin_ws:/workspace \
  sapphire:noetic
```
