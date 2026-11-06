# Sapphire ROS 1 Migration Design

Status: the Noetic adapter, launch files, and pinned toolchain image are implemented. A numeric parity threshold against ROS 2 has not been set.

## 1. Goal

Provide a native ROS 1 Noetic interface for Sapphire while keeping the
algorithm core shared with the ROS 2 interface.

The project should support:

- Existing ROS 1 robots without a ROS 2 runtime.
- Old LiDAR/IMU datasets stored as ROS 1 bags.
- Side-by-side ROS 1 and ROS 2 interface packages built from the same
  `sapphire` core.
- Identical algorithm behavior and configuration semantics across both
  interfaces.

The target is native ROS 1. A `ros1_bridge` deployment is not part of this
design because it requires both middlewares, adds another operational process,
and increases point-cloud transport overhead and failure modes.

## 2. Scope

In scope:

- `sapphire_ros1` catkin package.
- ROS message conversion for LiDAR, IMU, odometry, paths, maps, graph markers,
  and TF.
- Topic remapping, private parameters, roslaunch configuration, and RViz
  configuration.
- ROS 1 bag replay and in-node TUM trajectory export.
- A reproducible Noetic development and build image.

Out of scope:

- Changes to Sapphire odometry, mapping, registration, or pose-graph algorithms.
- Sensor driver implementation inside the ROS interface package.
- ROS 1 Melodic or Ubuntu 18.04 support.
- A fused runtime that links ROS 1 and ROS 2 into the same process.

## 3. Architecture

```text
                 +-------------------+
 ROS 1 messages  |   sapphire_ros1   |
 -----------------> thin adapter     |
                 +---------+---------+
                           |
                           v
                 +-------------------+
                 |      sapphire     |
                 | ROS-independent   |
                 | C++17 core        |
                 +---------+---------+
                           ^
                           |
                 +---------+---------+
 ROS 2 messages  |   sapphire_ros2   |
 -----------------> thin adapter     |
                 +-------------------+
```

Rules:

1. The core receives only `sapphire::ImuData`, `sapphire::PointCloud`, and
   `sapphire::Config`.
2. The core does not include any ROS 1 or ROS 2 header and does not use ROS
   build macros.
3. Each interface owns its own message types, launch files, parameters, build
   files, and container recipe.
4. Sensor-specific point-field detection remains in the interface layer.
5. Algorithm changes are made once in `sapphire` and become available to both
   interfaces after rebuilding.

The ROS 1 adapter should be a direct port of the existing thin ROS 2 adapter.
A new cross-middleware abstraction layer is not justified at the current size
of approximately 550 lines of adapter implementation.

## 4. Core API Contract

The ROS 1 wrapper uses the existing public core API:

```cpp
void pushImu(const sapphire::ImuData& imu);
void pushLidar(double stamp, const sapphire::PointCloudConstPtr& points);
OdometryResult latestResult() const;
std::optional<OdometryResult> latestPropagatedResult() const;
PointCloudConstPtr latestDeskewed() const;
MapTrajectory mapTrajectory() const;
```

The wrapper owns the `OdometryPipeline` instance and performs only:

- ROS message to core type conversion.
- Core result to ROS message conversion.
- ROS topic, parameter, TF, and callback lifecycle management.

## 5. ROS API Contract

Default topics and frames should match the ROS 2 semantic contract. Launch
arguments may override every external topic.

| Direction | Default topic | Message |
|---|---|---|
| Input | `/front_lidar` | `sensor_msgs/PointCloud2` |
| Input | `/front_lidar/imu` | `sensor_msgs/Imu` |
| Output | `/sapphire/odometry` | `nav_msgs/Odometry` |
| Output | `/sapphire/deskewed` | `sensor_msgs/PointCloud2` |
| Output | `/sapphire/trajectory` | `nav_msgs/Path` |
| Output | `/sapphire/pgo/map` | `sensor_msgs/PointCloud2` |
| Output | `/sapphire/pgo/graph` | `visualization_msgs/MarkerArray` |
| Output | `/tf` | `geometry_msgs/TransformStamped` |

Default frames:

- World: `map`
- LiDAR child: `lidar`

The node must preserve the input clock domain and publish output timestamps
with nanosecond precision.

## 6. Message and Build Mapping

| ROS 2 | ROS 1 |
|---|---|
| `rclcpp::Node` | `ros::NodeHandle` |
| `rclcpp::Publisher<T>` | `ros::Publisher` |
| `rclcpp::Subscription<T>` | `ros::Subscriber` |
| `rclcpp::TimerBase` | `ros::Timer` |
| `sensor_msgs::msg::PointCloud2` | `sensor_msgs::PointCloud2` |
| `sensor_msgs::msg::Imu` | `sensor_msgs::Imu` |
| `nav_msgs::msg::Odometry` | `nav_msgs::Odometry` |
| `nav_msgs::msg::Path` | `nav_msgs/Path` |
| `visualization_msgs::msg::MarkerArray` | `visualization_msgs/MarkerArray` |
| `tf2_ros::TransformBroadcaster(*node)` | `tf2_ros::TransformBroadcaster()` |
| `ament_cmake` | `catkin` |
| `colcon build` | `catkin build` |
| Python launch | roslaunch XML |
| `ament_index_cpp` package path | roslaunch substitution or `ros::package::getPath` |

No custom messages, services, or actions are required.

## 7. Queue and Latching Semantics

ROS 1 has no DDS QoS. Preserve the useful ROS 2 semantics with queue sizes and
latched publishers:

| Stream | ROS 1 behavior |
|---|---|
| LiDAR input | `queue_size=1` |
| IMU input | `queue_size=100`, enough to cover `OdometryPipeline::kImuCoverageWait` |
| Odometry output | `queue_size=10` |
| Deskewed cloud | `queue_size=1` |
| Trajectory path | `queue_size=1`, latched |
| PGO map | `queue_size=1`, latched |
| PGO graph | `queue_size=1`, latched |

The large point-cloud topics must not use an unbounded queue.

## 8. Callback Concurrency

This is the highest-risk adapter detail.

After the first scan, the core waits up to
`sapphire::OdometryPipeline::kImuCoverageWait` of wall time for IMU
measurements that cover the scan end. The IMU mutex is released during that
wait. A single-threaded `ros::spin()` cannot deliver the IMU, so the wait
expires and the scan is dropped.

Required behavior:

- Use at least two callback threads, such as `ros::AsyncSpinner(2)`.
- Allow LiDAR and IMU callbacks to execute concurrently.
- Do not allow two LiDAR callbacks to mutate the pipeline concurrently.
- Keep output publication away from callback member lifetimes that may end
  during shutdown.

The implementation must include a concurrency smoke test, not only a
single-message conversion test.

## 9. Parameters

Recommended private parameters:

| Parameter | Default |
|---|---|
| `~config_file` | Required; path to a core-owned algorithm TOML |
| `~world_frame` | `map` |
| `~lidar_frame` | `lidar` |
| `~imu_frame` | `imu` |
| `~pose_frame` | `lidar` (`lidar` or `imu`) |
| `~lidar_time_offset` | `0.0` s |
| `~imu_time_offset` | `0.0` s |
| `~trajectory_output` | empty; use TOML output path |

The launch file provides topic remappings rather than turning topic names into
C++ parameters. This keeps remapping behavior idiomatic for ROS 1.

## 10. PCL Policy

Noetic apt ships PCL 1.10, while small_gicp requires PCL 1.11 or newer.
Sapphire therefore uses PCL 1.14 from `/opt/sapphire-deps`.

The core must use one PCL installation consistently. The ROS 1 adapter:

- May use the header-only `pcl_conversions` conversion API.
- Must not link `pcl_ros` unless there is an actual runtime need.
- Must not instantiate ROS 1 PCL nodelets from the PCL 1.10 ABI.
- Must verify the resolved `PCL_DIR` during configuration.

Linking both the ROS 1 PCL 1.10 shared libraries and Sapphire's PCL 1.14
libraries is a prohibited configuration.

## 11. Dependency Findings

The earlier `sapphire:noetic` image was inspected and tested. Its ROS 1 package
set is complete, but its algorithm dependency stack is not reproducible:

- System Eigen is 3.3.7. Current preintegration headers require the Eigen 3.4
  `Eigen::Vector<T, N>` alias.
- The image has a custom PCL 1.14 installation and a separate apt PCL 1.10
  installation.
- GTSAM came from an obsolete Launchpad PPA.
- The PPA is no longer reachable, so `apt-get update` fails and the old image
  cannot be rebuilt from its original recipe.
- The Pose Graph test deterministically aborted inside the installed
  `libgtsam.so` with `double free or corruption`.
- The same source and Pose Graph test passed in the ROS 2 Humble image.

The new image therefore builds Eigen, GTSAM, PCL, small_gicp, and toml++ from
pinned source revisions into an isolated prefix.

## 12. Docker Boundary

The Dockerfile lives in this ROS interface package because dependency needs
may differ between interface layers.

The image is a development toolchain image. It does not copy a user's Sapphire
workspace into the image. The workspace is mounted at runtime so both
interfaces can continue to track their own Git histories.

See [DOCKER.md](DOCKER.md) for build and usage instructions.

## 13. Acceptance Criteria

Implementation is complete when all of the following hold:

1. The pinned toolchain image builds successfully from a clean cache.
2. `catkin build sapphire_ros1` succeeds for the current worktree.
3. The core test suite passes in the Noetic image, including Pose Graph.
4. The ROS 1 node starts with `roslaunch`.
5. LiDAR, IMU, odometry, TF, deskewed cloud, trajectory, PGO map, and graph
   topics have the documented types, frames, and timestamps.
6. A ROS 1 bag replay completes without deadlock, crash, or unbounded queue
   growth.
7. ROS 1 and ROS 2 produce comparable trajectories on the same recording.
8. The node writes a valid final TUM trajectory on clean shutdown, with an
   optional launch-time output-path override.
9. Build dependencies are declared in `package.xml`.
10. No ROS header or ROS build tool is added to the `sapphire` core.

## 14. Test Plan

### Core tests

Run the complete Sapphire CTest suite in the pinned image before adapter work.

### Conversion tests

- PointCloud2 fields: `offset_time`, `t`, `timestamp`, `time`, `time_stamp`.
- Relative and absolute point timestamps.
- Missing and unsupported time fields.
- IMU acceleration, angular velocity, and nanosecond timestamps.
- Empty and malformed clouds.

### Integration tests

- ROS 1 bag replay with `/clock`.
- Topic rate and type checks.
- TF connectivity from `map` to `lidar`.
- IMU initialization during stationary data.
- LiDAR callback waiting up to `kImuCoverageWait` for scan-end IMU coverage. One callback thread drops the scan instead of stalling IMU delivery.
- PGO visualization subscription behavior.
- Shutdown trajectory dump.

### ROS 1/ROS 2 parity

Record the same source data in both middlewares, or convert the bag once and
replay equivalent inputs. Compare:

- Final trajectory pose.
- Path length.
- Message timestamps.
- Deskewed cloud count.
- Keyframe and loop-closure counts.

Exact floating-point equality is not required. The acceptance thresholds
should be defined when the first dataset comparison is available.

## 15. Implementation Sequence

Implementation progress:

1. Done: `package.xml`, catkin `CMakeLists.txt`, and package metadata.
2. Done: C++ wrapper and conversion tests.
3. Done: roslaunch, RViz, and in-node TUM export.
4. Done: pinned Docker image in `docker/Dockerfile`.
5. Dataset replay is documented in the core repository's `docs/DATASET_REPLAY.md`.
6. Open: choose numeric acceptance thresholds for a ROS 1 / ROS 2 comparison on the same recording. Exact floating-point equality is not required.

## 16. Implemented Review Decisions

The implementation adopts these interface decisions:

1. Support only ROS 1 Noetic and Ubuntu 20.04.
2. Keep `/front_lidar` and `/front_lidar/imu` as launch defaults.
3. Use one node binary named `sapphire_ros_node`.
4. Use a non-root container user matching host UID/GID 1000 by default.
5. Keep Docker images per interface package, using a shared dependency-version
   policy documented in both packages.
6. Do not add sensor drivers or Livox SDK to this interface image; consume
   generic `PointCloud2` and `Imu`.

