# Sapphire ROS 1 Docker Toolchain

## Purpose

The image provides a reproducible ROS 1 Noetic development toolchain for the
shared Sapphire core and the native ROS 1 interface.

It is not an application image. User source remains in a mounted workspace, so
the same checkout can be built on the host or in the container without copying
Git history into an image layer.

## Pinned Toolchain

| Component | Version | Revision |
|---|---|---|
| Ubuntu | 20.04 | ROS Noetic base image |
| Eigen | 3.4.0 | `3147391d946bb4b6c68edd901f2add6ac1f31f8c` |
| PCL | 1.14.1 | `5f608cfb5397fe848c7d61c4ae5f5b1ab760ba80` |
| GTSAM | 4.3.0 | `71a25ca36c084cbad1f872e812d6d97fbadfdb05` |
| small_gicp | 1.0.1 | `57c1106daf83c2c79ee0c58a9c7ed0032298ff4e` |
| toml++ | 3.4.0 | `30172438cee64926dc41fdd9c11fb3ba5b2ba9de` |

The source revisions are verified after cloning. If an upstream tag is moved,
the build fails instead of silently changing the dependency graph.

GTSAM 4.3.0 has an upstream CMake edge case that passes an empty list to
`list(REMOVE_ITEM)` on CMake 3.16. The image applies one narrow patch,
`docker/patches/gtsam-4.3.0-empty-list.patch`, that guards the two empty-list
calls and excludes GTSAM's unused constrained/certifiable modules. It does not
change the solver behavior used by Sapphire.

GTSAM is built with `GTSAM_WITH_TBB=OFF` because GTSAM 4.3 expects the oneTBB
headers while Ubuntu 20.04 provides TBB 2020 classic. Sapphire uses OpenMP for
its own parallel work and does not require GTSAM's TBB backend.

Dependencies are installed under:

```text
/opt/sapphire-deps
```

This isolated prefix prevents CMake from selecting the Ubuntu PCL 1.10 package
or its Eigen 3.3 installation for Sapphire.

## Build

Build from the `sapphire_ros1` package root:

```bash
cd src/sapphire_ros1
docker build \
  -f docker/Dockerfile \
  -t sapphire:noetic \
  .
```

Use host UID/GID defaults of 1000:

```bash
docker build \
  -f docker/Dockerfile \
  --build-arg USER_UID="$(id -u)" \
  --build-arg USER_GID="$(id -g)" \
  -t sapphire:noetic \
  .
```

An optional ROS base image override is available, but the image must be a
Noetic focal image:

```bash
docker build \
  -f docker/Dockerfile \
  --build-arg ROS_BASE_IMAGE=ros:noetic-ros-base-focal \
  -t sapphire:noetic \
  .
```

Ubuntu and ROS package downloads use the Tsinghua mirror by default:

```text
https://mirrors.tuna.tsinghua.edu.cn/ubuntu
https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu
```

Override the mirrors for networks where the official repositories are faster:

```bash
docker build \
  -f docker/Dockerfile \
  --build-arg UBUNTU_MIRROR=http://archive.ubuntu.com/ubuntu \
  --build-arg ROS_MIRROR=http://snapshots.ros.org/noetic/final/ubuntu \
  -t sapphire:noetic \
  .
```

The ROS 1 archive key already present in the base image is retained when the
repository URLs are rewritten. The base image's snapshot-only key is
supplemented with the ROS archive key whose primary fingerprint is:

```text
C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

The build verifies this fingerprint before enabling the Tsinghua ROS mirror.

For release or CI use, replace the tag with an immutable image digest after the
base image has been validated.

## Run

Mount the complete workspace, not only the interface package:

```bash
docker run --rm -it \
  -v /path/to/sapphire_ws:/workspace \
  -v /path/to/datasets:/data:ro \
  sapphire:noetic
```

If the host UID/GID differs from 1000, rebuild with the matching build
arguments. This keeps generated `build/` and `devel/` files editable on the
host.

## Build Sapphire

Inside the container:

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

The ROS 1 package finds the sibling core at:

```text
/workspace/src/sapphire
```

The core remains a plain CMake static library. It is added as a subdirectory by
the ROS 1 package and does not need a catkin manifest.

## Verify Dependencies

Inside the image:

```bash
source /opt/ros/noetic/setup.bash
rosversion -d
grep -E 'EIGEN_(WORLD|MAJOR|MINOR)_VERSION' \
  /opt/sapphire-deps/include/eigen3/Eigen/src/Core/util/Macros.h
grep -E 'PCL_VERSION' \
  /opt/sapphire-deps/include/pcl-*/pcl/pcl_config.h
grep -E 'GTSAM_VERSION_(MAJOR|MINOR|PATCH)' \
  /opt/sapphire-deps/include/gtsam/config.h
```

Configure a clean core build with:

```bash
cmake -S /workspace/src/sapphire \
  -B /tmp/sapphire-build \
  -DBUILD_TESTING=ON \
  -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/sapphire-build -j"$(nproc)"
cd /tmp/sapphire-build
ctest --output-on-failure
```

Expected first milestone:

- Eigen is 3.4.0.
- CMake resolves PCL from `/opt/sapphire-deps`.
- The core configures without fetching toml++.
- The Pose Graph test no longer aborts inside `libgtsam.so`.

## Runtime Devices

The image does not require GPU access for the current Sapphire core.

If a ROS 1 driver needs USB or CAN devices, pass only the required device or
use host networking. Do not grant broad privileged access by default.

Example for a specific serial device:

```bash
docker run --rm -it \
  --device=/dev/ttyUSB0 \
  -v /path/to/sapphire_ws:/workspace \
  sapphire:noetic
```

## GUI

For RViz, forward X11 using the host's existing X11 socket:

```bash
xhost +local:docker
docker run --rm -it \
  -e DISPLAY="$DISPLAY" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /path/to/sapphire_ws:/workspace \
  sapphire:noetic
```

## ROS 1 Lifecycle

ROS 1 Noetic is end-of-life. The image uses the available Noetic package
archive for legacy support. Security and package availability should be
monitored separately from the ROS 2 development path.

The long-term repository strategy remains:

- ROS 2 is the primary actively supported interface.
- ROS 1 remains available as a native compatibility interface for legacy users
  and datasets.
- Both interfaces build the same algorithm core.

