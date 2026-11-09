# HILTI bags on ROS 1

These launch files set the topics and frames for one rig and can play the bag. `config_file` is still required. Other datasets, and the same bags on ROS 2, are in [DATASET_REPLAY.md](https://github.com/wagdeer/sapphire/blob/master/docs/DATASET_REPLAY.md).

Handheld 2023. The launch file applies the LiDAR time offset used with this rig.

```bash
roslaunch sapphire_ros1 hilti_2023_handheld.launch \
  config_file:=/path/to/sapphire/cfg/hilti_2023_handheld.toml \
  bag:=/path/to/site1_handheld_1.bag \
  trajectory_output:=/path/to/trajectory.tum
```

Handheld 2022 uses the same topics, `/hesai/pandar` and `/alphasense/imu`, with the time offset left at 0.

```bash
roslaunch sapphire_ros1 hilti_2022_handheld.launch \
  config_file:=/path/to/sapphire/cfg/hilti_2022_handheld.toml \
  bag:=/path/to/exp03_construction_stairs.bag \
  trajectory_output:=/path/to/trajectory.tum
```

Robot. Topics are `/rslidar_points` and `/imu/data`.

```bash
roslaunch sapphire_ros1 hilti_2023_robot.launch \
  config_file:=/path/to/sapphire/cfg/hilti_2023_robot.toml \
  bag:=/path/to/site2_robot_2.bag \
  trajectory_output:=/path/to/trajectory.tum
```

`bag_required:=true` exits the launch when the bag finishes, which writes the TUM file. `rviz:=true` needs a display. `launch/display.rviz` shows the grid, odometry, trajectory, deskewed cloud, PGO map, and PGO graph.

Build the workspace first. See [README.md](../README.md) and [DOCKER.md](DOCKER.md).
