# HILTI 2023 Reproduction Guide

This is the restart-safe reproduction entry point for the ROS 1 HILTI 2023
work. Start here after rebooting the computer.

The workspace is host-mounted into the container. Source, build products,
trajectories, and evaluation results therefore survive container recreation and
host reboots.

## Inputs and expected layout

Host dataset root:

```text
/mnt/e/dataset/hilti-2023
```

Container dataset root:

```text
/data
```

Primary tuning sequence:

```text
Host:      /mnt/e/dataset/hilti-2023/rosbags/site1_handheld_5.bag
Container: /data/rosbags/site1_handheld_5.bag
Ground truth:
           /mnt/e/dataset/hilti-2023/ground_truth/site1_handheld_5.txt
```

Cross-validation sequence:

```text
Host:      /mnt/e/dataset/hilti-2023/rosbags/site1_handheld_2.bag
Container: /data/rosbags/site1_handheld_2.bag
Ground truth:
           /mnt/e/dataset/hilti-2023/ground_truth/site1_handheld_2.txt
```

The `site1_handheld_5.bag` file is about 17.1 GB and has about 159 seconds of
playback at `rate:=1.0`.

## Container and image provenance

Start from the workspace root:

```bash
cd /home/bindeer/git/sapphire_ws
docker compose -f docker-compose.ros1.yml up -d
docker ps --filter name=sapphire_ros1_dev
```

The expected container is:

```text
sapphire_ros1_dev
```

The expected ROS 1 image tag is:

```text
sapphire:noetic
```

The offline evaluator image is:

```text
sapphire-slam-eval:0.1.0
```

Record current image identities after a reboot:

```bash
docker image inspect sapphire:noetic \
  --format '{{.Id}} {{.Created}}'
docker image inspect sapphire-slam-eval:0.1.0 \
  --format '{{.Id}} {{.Created}}'
```

The images observed on 2026-11-01 were:

```text
sapphire:noetic
sha256:6d69740833df5e18a1df918191b4e1b7a9f0fd57877462ceda034887808d2a78

sapphire-slam-eval:0.1.0
sha256:dff1a3a797c4118b82ed661604b8a82a678fcb07ed7dddf440b1ba5cdcb7a81f
```

If an image is missing, rebuild it according to `DOCKER.md`.

## Build and test

Use the dedicated ROS 1 Catkin spaces:

```text
/workspace/build_ros1
/workspace/devel_ros1
```

Do not use the older `/workspace/build` and `/workspace/install` spaces for the
ROS 1 adapter.

Run the build and tests in the container:

```bash
docker exec -it sapphire_ros1_dev bash -lc '
  source /opt/ros/noetic/setup.bash
  cd /workspace
  catkin build sapphire_ros1 --no-deps
  source /workspace/devel_ros1/setup.bash
  catkin test sapphire_ros1 --no-deps
'
```

Expected last verified result:

```text
6/6 C++ conversion tests passed
Catkin summary: 12 tests, 0 errors, 0 failures, 0 skipped
```

## Interactive replay with RViz

Enter the container:

```bash
docker exec -it sapphire_ros1_dev bash
```

Source the ROS 1 environment:

```bash
source /opt/ros/noetic/setup.bash
source /workspace/devel_ros1/setup.bash
cd /workspace
```

Create a new output directory so existing results are not overwritten:

```bash
RUN_DIR=/workspace/results/hilti_2023/reproduction/$(date +%Y%m%d_%H%M%S)
mkdir -p "$RUN_DIR"
```

Run the tuned handheld profile with the lightweight visual-check RViz config:

```bash
roslaunch sapphire_ros1 hilti_2023_handheld.launch \
  config_file:=/workspace/src/sapphire/cfg/sapphire_hilti_2023_handheld_tuned.toml \
  bag:=/data/rosbags/site1_handheld_5.bag \
  rate:=1.0 \
  rviz:=true \
  rviz_config:=/workspace/src/sapphire_ros1/launch/visual_check.rviz \
  bag_required:=true \
  trajectory_output:="$RUN_DIR/trajectory.tum" \
  2>&1 | tee "$RUN_DIR/run.log"
```

The lightweight profile displays:

- `/alphasense/cam0/image_raw`
- `/sapphire/deskewed`
- `/sapphire/trajectory`
- `/sapphire/odometry`
- the fixed grid

It intentionally excludes the heavier `/sapphire/pgo/map` and
`/sapphire/pgo/graph` displays. The full profile remains available as
`launch/display.rviz`.

Verify live data from a second terminal:

```bash
docker exec sapphire_ros1_dev bash -lc '
  source /opt/ros/noetic/setup.bash
  timeout 5s rostopic hz /sapphire/deskewed
  timeout 5s rostopic hz /alphasense/cam0/image_raw
  timeout 5s rostopic hz /sapphire/trajectory
'
```

Observed healthy rates on 2026-11-01:

```text
Camera:       about 58 Hz
Deskewed:     about 10-15 Hz
Trajectory:   continuous once IMU initialization completes
```

The complete trajectory is written when the node receives a normal shutdown
signal. Do not treat a force-killed container as a completed experiment.

## Headless experiment run

For a complete run that does not need RViz:

```bash
roslaunch sapphire_ros1 hilti_2023_handheld.launch \
  config_file:=/workspace/src/sapphire/cfg/sapphire_hilti_2023_handheld_tuned.toml \
  bag:=/data/rosbags/site1_handheld_5.bag \
  rate:=1.0 \
  rviz:=false \
  bag_required:=true \
  trajectory_output:="$RUN_DIR/trajectory.tum" \
  2>&1 | tee "$RUN_DIR/run.log"
```

Wait for:

```text
[sapphire_ros1] wrote N TUM poses to ...
```

For the selected E26 profile, the completed reference run had:

```text
1588 poses
158.76 seconds
```

## Profile selection

The launch default is the aggressive E26 profile:

```text
/workspace/src/sapphire/cfg/sapphire_hilti_2023_handheld_tuned.toml
```

Key values:

```toml
[odometry]
voxel_size = 0.05
fusion = "eskf"

[odometry.submap]
splitting_distance = 0.75
splitting_rotation = 0.02
max_keyframes = 5

[registration]
type = "VGICP"

[imu.init]
convergence_gyro_std = 0.0065
```

E26 works best on the staircase sequence `site1_handheld_5`, but it is
over-specialized. On `site1_handheld_2`, the conservative E07 profile is
better. To reproduce E07:

```bash
roslaunch sapphire_ros1 hilti_2023_handheld.launch \
  bag:=/data/rosbags/site1_handheld_2.bag \
  config_file:=/workspace/results/hilti_2023/experiments/configs/e07_submap_075.toml \
  rate:=1.0 \
  rviz:=false \
  bag_required:=true \
  trajectory_output:="$RUN_DIR/trajectory.tum"
```

## Evaluation

Evaluate a completed trajectory with the separate `eval/` project. The ROS
interface package intentionally ships no trajectory-evaluation scripts:

```bash
cd /home/bindeer/git/sapphire_ws/eval
docker compose run --rm eval run \
  --config configs/hilti_2023.yaml \
  --output /results/hilti_2023
```

The evaluator applies:

- handheld pole-tip transform `[0.059, -0.00855, 0.1964] m`
- maximum 2 second estimate/reference association difference
- SE(3) Umeyama alignment
- no scale correction

Expected E26 result:

```text
matched_poses: 3
ate_rmse_m:    0.06144193058912346
ate_max_m:     0.06809399648156952
```

For `site1_handheld_2`, use its matching ground truth and the applicable
trajectory. The conservative E07 result is:

```text
matched_poses: 4
ate_rmse_m:    0.0393032653894739
ate_max_m:     0.04980331534005068
```

HILTI reference files contain sparse control poses. Results with different
matched-pose counts are not directly equivalent experiments.

## RViz hang and container recovery

On 2026-11-01, the full `display.rviz` profile consumed about 80% CPU in RViz
and a normal Ctrl-C did not terminate the RViz process immediately. The window
could appear dead. The lightweight profile reduced observed RViz CPU use to
about 26%.

If RViz still cannot be closed:

1. Stop it normally once with Ctrl-C.
2. If the RViz process remains alive, stop only the ROS 1 container:

```bash
docker stop --timeout 5 sapphire_ros1_dev
```

3. Restart it with the existing Compose service:

```bash
cd /home/bindeer/git/sapphire_ws
docker compose -f docker-compose.ros1.yml up -d
```

4. Verify that no stale ROS/RViz process remains:

```bash
docker exec sapphire_ros1_dev bash -lc \
  "pgrep -af 'rviz|rosmaster|roscore|roslaunch|rosbag|sapphire_ros_node' || true"
```

The workspace is bind-mounted, so container restart does not delete source or
completed results. It can, however, prevent a partial trajectory from being
written because the ROS node normally saves its TUM file during graceful
shutdown.

## Experiment records

Formal ranking and analysis:

```text
/workspace/results/hilti_2023/experiments/summary.md
/workspace/results/hilti_2023/experiments/summary.csv
/workspace/results/hilti_2023/experiments/summary.json
```

Effective configs:

```text
/workspace/results/hilti_2023/experiments/configs/
```

Per-run artifacts:

```text
/workspace/results/hilti_2023/experiments/runs/<experiment>/
├── config.toml
├── run.log
├── trajectory.tum
└── eval.json
```

There are 31 formal run directories, each with a trajectory and evaluation
result. Interactive visualization checks are stored separately under:

```text
/workspace/results/hilti_2023/visual_check/
```

The visual-check attempts are not part of the formal 31-run ranking.

## Related documents

- `README.md`: ROS 1 package overview and basic commands
- `DOCKER.md`: image build and pinned dependency details
- `MIGRATION.md`: ROS 1 interface architecture
- `HILTI_TUNING.md`: tuning experiments, calibration audit, and conclusions
