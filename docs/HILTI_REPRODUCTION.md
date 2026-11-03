# HILTI 2023 handheld replay

ROS 1 procedure for the HILTI 2023 handheld recordings. Build the image and
the workspace with [README.md](../README.md) and [DOCKER.md](DOCKER.md) first.
The core repository must be a sibling of this package.

Published scores live in the core benchmark table, not in the tuning log:

```text
sapphire/docs/benchmarks/benchmark_table.md
```

The Site 1 handheld mean there is 0.058 m. Site 1 Handheld 5 in that table is
0.045 m from a 1571-pose trajectory. [HILTI_TUNING.md](HILTI_TUNING.md) records
an earlier parameter search. Its selected run, E26, scored 0.061 m on the same
sequence with 1588 poses. Those are two runs. Cite the benchmark table for
the published number.

## Dataset layout

Point `HILTI_DIR` at the dataset root:

```text
$HILTI_DIR/rosbags/site1_handheld_5.bag
$HILTI_DIR/ground_truth/site1_handheld_5.txt
```

`site1_handheld_5.bag` is about 17 GB and plays for about 159 s at `rate:=1.0`.
The same layout is used for the other `site1_handheld_*.bag` files.

## Run

From a shell where this workspace is built and sourced:

```bash
export HILTI_DIR=/path/to/hilti-2023
export RUN_DIR=/path/to/results/hilti_2023/site1_handheld_5
mkdir -p "$RUN_DIR"

roslaunch sapphire_ros1 hilti_2023_handheld.launch \
  config_file:=/path/to/sapphire/cfg/sapphire_hilti_2023_handheld.toml \
  bag:="$HILTI_DIR/rosbags/site1_handheld_5.bag" \
  rate:=1.0 \
  rviz:=false \
  bag_required:=true \
  trajectory_output:="$RUN_DIR/trajectory.tum"
```

The launch file subscribes to `/hesai/pandar` and `/alphasense/imu`, sets
`pose_frame` to `imu`, and applies the handheld LiDAR time offset
`0.0015846550361175184` s. An empty `config_file` is not a valid default;
pass the profile explicitly. The copy in `cfg/` is the profile selected by
the tuning log.

Wait until the log reports that the TUM file was written. That happens on a
normal shutdown, which this launch performs when `bag_required:=true` and the
bag finishes. A killed container does not write the file.

For a live view, add `rviz:=true`. The default config is
`launch/display.rviz`: grid, odometry, trajectory, deskewed cloud, PGO map,
and PGO graph. `launch/visual_check.rviz` shows the same displays except the
PGO graph. Neither file displays a camera.

## Evaluation

This package does not ship an evaluator. The published HILTI 2023 numbers use:

- the handheld pole-tip translation `[0.059, -0.00855, 0.1964]` m, applied to
  the estimated IMU trajectory
- a maximum 2 s timestamp difference
- SE(3) Umeyama alignment, no scale correction
- translation only, because the reference orientations are constant

A score from 3 control points is not comparable to a score from 4 control
points, and neither is a dense-trajectory ATE. Site 2 results are reported
beside the Site 1 mean in the benchmark table and are not included in 0.058 m.

## Related documents

- [README.md](../README.md): build and a generic launch
- [DOCKER.md](DOCKER.md): pinned Noetic toolchain
- [HILTI_TUNING.md](HILTI_TUNING.md): 2026-11-01 parameter search
- [INTERFACE_CONTRACT.md](INTERFACE_CONTRACT.md): topics, frames, and QoS
