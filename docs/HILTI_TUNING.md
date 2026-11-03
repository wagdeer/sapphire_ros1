# HILTI 2023 Drift Tuning

Date: 2026-11-01

This is the parameter-search log, not the published benchmark. Replay commands
are in [HILTI_REPRODUCTION.md](HILTI_REPRODUCTION.md). The published Site 1
numbers are in `sapphire/docs/benchmarks/benchmark_table.md`.

On `site1_handheld_5`, this log's selected run E26 has ATE RMSE 0.061 m and
1588 poses. The benchmark table lists 0.045 m for a later 1571-pose trajectory
of the same sequence. Do not cite the two as one measurement. Paths under
`/workspace/results/` are local artifacts and are not part of this repository.

## Scope

Tuning sequence:

```text
/datasets/hilti-2023/rosbags/site1_handheld_5.bag
```

Validation sequence:

```text
/datasets/hilti-2023/rosbags/site1_handheld_2.bag
```

Both use the HILTI handheld calibration. All 31 raw experiment records are
under:

```text
/workspace/results/hilti_2023/experiments/
```

The generated ranking is:

- `summary.md`
- `summary.csv`
- `summary.json`

Each run directory contains the effective TOML config, ROS launch log,
full-resolution TUM trajectory, and corrected evo evaluation result.

## Calibration audit

- LiDAR: Hesai PandarXT-32.
- IMU: Alphasense Core.
- LiDAR input fields: `x/y/z:float32`, `intensity:float32`,
  `timestamp:float64`, `ring:uint8`.
- LiDAR point timestamps are absolute seconds.
- Core extrinsic is the calibration file's `T_imu0_lidar0`.
- Translation: `[-0.006950465, -0.006713482, 0.049893461] m`.
- Rotation quaternion `xyzw`:
  `[0.709439749, -0.704765131, 0.001135775, -0.000250946]`.
- The calibrated rotation determinant is 1 and orthogonality error is about
  `2.2e-16`.
- LiDAR-to-IMU time shift is applied as
  `t_imu = t_lidar + 0.001584655`.
- IMU noise density and random-walk parameters come directly from
  `calibration_handheld/calibration_imu.yaml`.
- Handheld evaluation applies the dataset's pole-tip transform
  `[0.059, -0.00855, 0.1964] m` to the estimated IMU trajectory.

No camera intrinsics are used by the LiDAR-inertial pipeline.

## Evaluation protocol

The HILTI reference files contain sparse control poses, not continuous
trajectories. Evaluation therefore follows the challenge-style procedure:

1. Convert estimated LiDAR poses to IMU poses in the node.
2. Apply the handheld pole-tip transform to the estimate.
3. Associate estimate and reference with a maximum 2 s timestamp difference.
4. Apply SE(3) Umeyama alignment without scale correction.
5. Compute evo translation-part APE.

The implementation reproduces the alignment behavior used by the formal
`eval/` engine. Do not compare scores with different matched-pose counts as
though they use the same coverage.

## Retracted measurements

An intermediate version of the external evaluator called evo APE before
applying Umeyama alignment. It incorrectly reported 31-36 m errors and
produced an invalid ranking.

Retracted: all numerical rankings produced before the alignment fix.

Replacement: every experiment was re-evaluated with explicit SE(3) alignment.
The corrected values are in `experiments/summary.md`.

## Key experiments on site1_handheld_5

| Experiment | Change from preceding best | ATE RMSE [m] | ATE max [m] | Result |
|---|---|---:|---:|---|
| E01 | initialization gyro threshold `0.005→0.0065` | 0.209 | 0.289 | initialization no longer times out; all 3 GT poses matched |
| E07 | submap distance `1.5→0.75 m` | 0.186 | 0.222 | first clear improvement |
| E19 | submap rotation `0.785→0.30 rad` | 0.169 | 0.208 | further improvement |
| E20 | rotation `0.30→0.20 rad` | 0.164 | 0.214 | small RMSE improvement |
| E23 | rotation `0.20→0.02 rad` | 0.063 | 0.088 | large improvement |
| E26 | local submap frames `10→5` | **0.061** | **0.068** | selected profile |

Experiments that did not improve the selected path:

- Larger point clouds alone: 0.25, 0.10, and 0.05 m voxel combinations.
- Relaxing the PGO loop acceptance fit threshold.
- More PGO iterations and wider PGO correspondence distance.
- Observer fusion instead of ESKF.
- Online IMU bias updates.
- Full GICP pose injection.
- Disabling Hessian-derived covariance.
- Increasing or decreasing the nominal submap distance away from 0.75 m.

Many PGO candidates had high overlap but were rejected because loop GICP did
not converge or exceeded the fitness/residual gates. No loop was committed in
the selected run, so the improvement came from the frontend/submap behavior,
not from accepting speculative loops.

## Cross-sequence validation

| Profile | site1_handheld_5 RMSE [m] | site1_handheld_2 RMSE [m] | site1_handheld_2 GT matches |
|---|---:|---:|---:|
| E01: init fix, 1.5 m / 0.785 rad | 0.209 | 0.047 | 4 |
| E07: 0.75 m / 0.785 rad | 0.186 | 0.039 | 4 |
| E19: 0.75 m / 0.30 rad | 0.169 | 0.054 | 4 |
| E26: 0.75 m / 0.02 rad, 5 frames | **0.061** | 0.136 | 4 |

E26 gives the best two-sequence mean and worst-case error among the tested
profiles, but its aggressive rotation threshold is sensitive to scene/route.
The more conservative E07 is faster and slightly better on the validation
sequence, while E26 is substantially better on the staircase sequence.

## Code audit

The following core paths were reviewed after parameter tuning:

- LiDAR preprocessing and crop box.
- Deskew timestamp grouping and IMU interpolation.
- Extrinsic application in deskew and pose conversion.
- IMU initialization and gravity alignment.
- ESKF prediction/correction and bias handling.
- GICP pose gating.
- PGO loop cloud construction, registration, residual gating, and ISAM2
  commit path.

No code defect was found that explains the remaining error better than the
sparse-reference and configuration sensitivity above. One real code defect was
found in the standalone HILTI evaluation helper: missing Umeyama alignment.
That defect was fixed and all measurements were regenerated.

One additional invalid run was identified: E11 used a 1.0 m frontend
correspondence radius and lost the final 17.6 s of the bag. It is marked
`Full coverage = no` and is not considered an improvement.

## Selected configuration

The selected handheld profile is:

```text
src/sapphire/cfg/sapphire_hilti_2023_handheld.toml
```

Important values:

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

[pgo]
lidar_max_iterations = 64
```

`hilti_2023_handheld.launch` does not bake in a config path. Pass
`cfg/sapphire_hilti_2023_handheld.toml` as `config_file`. That file is the
profile selected here. An earlier profile is not kept beside it.

## Interactive visual-check attempts

These attempts were for operator inspection only and are not part of the 31-run
formal ranking.

- `e32_rviz_site1_handheld_5` used `display.rviz`. The node saved 669 poses
  when interrupted, so the trajectory is partial and was not evaluated. That
  RViz process consumed about 80% CPU and did not exit promptly after Ctrl-C.
- `e33_rviz_light_site1_handheld_5` used `visual_check.rviz`, which reduced
  observed RViz CPU use to about 26%. The container was force-stopped while
  the bag was still playing, so no final trajectory was written.
- Both sessions were cleared by restarting the ROS 1 container. Source and
  completed results on the mounted workspace were kept.

`display.rviz` shows the grid, odometry, trajectory, deskewed cloud, PGO map,
and PGO graph. `visual_check.rviz` shows the same displays except the PGO
graph. Neither file shows a camera.

## Remaining work

- Evaluate the selected profile on additional HILTI sites and robot sequences.
- Confirm whether the 0.02 rad rotation threshold causes excessive keyframe
  churn on longer recordings.
- Add a continuous or denser reference trajectory if available; three sparse
  control points are insufficient for a robust statistical conclusion.
- PGO currently contributes no accepted loops on this sequence. This remains a
  separate opportunity for long-trajectory correction.
