# Scenario results

Generated 2026-11-13 07:22 by `scenarios/run_all.sh` (seeds 1, 2, 3, 4, 5; headless fast mode).

| scenario | seed 1 | seed 2 | seed 3 | seed 4 | seed 5 | pass rate |
|---|---|---|---|---|---|---|
| gas_ahead | PASS | PASS | PASS | PASS | PASS | 5/5 |
| gas_dropout | PASS | PASS | PASS | PASS | PASS | 5/5 |
| mine | PASS | PASS | PASS | PASS | PASS | 5/5 |
| obstacles | PASS | PASS | PASS | PASS | PASS | 5/5 |
| pit_ahead | PASS | PASS | PASS | PASS | PASS | 5/5 |
| ramp | PASS | PASS | PASS | PASS | PASS | 5/5 |
| roof_fall | PASS | PASS | PASS | PASS | PASS | 5/5 |
| side_pit | PASS | PASS | PASS | PASS | PASS | 5/5 |

**Overall: 40/40 runs passed.**

## Failed checks

None.

## Per-run details

| run | sim s | path m | collisions | fell | final nav | hazards seen | images |
|---|---|---|---|---|---|---|---|
| gas_ahead_s1 | 38.91 | 3.161 | 0 | False | HOLD | SAFE CLEAR, CAUTION GAS_ELEVATED, DANGER GAS_BUILDUP | 10 |
| gas_ahead_s2 | 38.91 | 3.089 | 0 | False | HOLD | SAFE CLEAR, CAUTION GAS_ELEVATED, DANGER GAS_BUILDUP | 10 |
| gas_ahead_s3 | 39.01 | 3.158 | 0 | False | HOLD | SAFE CLEAR, CAUTION GAS_ELEVATED, DANGER GAS_BUILDUP | 10 |
| gas_ahead_s4 | 39.01 | 3.162 | 0 | False | HOLD | SAFE CLEAR, CAUTION GAS_ELEVATED, DANGER GAS_BUILDUP | 10 |
| gas_ahead_s5 | 38.91 | 3.129 | 0 | False | HOLD | SAFE CLEAR, CAUTION GAS_ELEVATED, DANGER GAS_BUILDUP | 10 |
| gas_dropout_s1 | 16.76 | 2.779 | 0 | False | CRUISE | SAFE CLEAR, CAUTION STALE_DATA | 0 |
| gas_dropout_s2 | 16.56 | 2.697 | 0 | False | CRUISE | SAFE CLEAR, CAUTION STALE_DATA | 0 |
| gas_dropout_s3 | 16.78 | 2.77 | 0 | False | CRUISE | SAFE CLEAR, CAUTION STALE_DATA | 0 |
| gas_dropout_s4 | 16.78 | 2.771 | 0 | False | CRUISE | SAFE CLEAR, CAUTION STALE_DATA | 0 |
| gas_dropout_s5 | 16.6 | 2.731 | 0 | False | CRUISE | SAFE CLEAR, CAUTION STALE_DATA | 0 |
| mine_s1 | 90.86 | 12.399 | 0 | False | HOLD | SAFE CLEAR, SAFE OBSTACLE, CAUTION FLOOR_LOST, CAUTION GAS_ELEVATED, DANGER GAS_BUILDUP | 15 |
| mine_s2 | 90.51 | 12.317 | 0 | False | HOLD | SAFE CLEAR, CAUTION FLOOR_LOST, SAFE OBSTACLE, CAUTION GAS_ELEVATED, DANGER GAS_BUILDUP | 15 |
| mine_s3 | 90.86 | 12.384 | 0 | False | HOLD | SAFE CLEAR, SAFE OBSTACLE, CAUTION FLOOR_LOST, CAUTION GAS_ELEVATED, DANGER GAS_BUILDUP | 15 |
| mine_s4 | 90.26 | 12.38 | 0 | False | HOLD | SAFE CLEAR, CAUTION FLOOR_LOST, SAFE OBSTACLE, CAUTION GAS_ELEVATED, DANGER GAS_BUILDUP | 15 |
| mine_s5 | 90.51 | 12.361 | 0 | False | HOLD | SAFE CLEAR, SAFE OBSTACLE, CAUTION FLOOR_LOST, CAUTION GAS_ELEVATED, DANGER GAS_BUILDUP | 15 |
| obstacles_s1 | 28.52 | 6.626 | 0 | False | CRUISE | SAFE CLEAR | 0 |
| obstacles_s2 | 27.72 | 6.531 | 0 | False | CRUISE | SAFE CLEAR | 0 |
| obstacles_s3 | 28.23 | 6.626 | 0 | False | CRUISE | SAFE CLEAR, SAFE OBSTACLE | 0 |
| obstacles_s4 | 28.17 | 6.622 | 0 | False | CRUISE | SAFE CLEAR | 0 |
| obstacles_s5 | 27.82 | 6.565 | 0 | False | CRUISE | SAFE CLEAR | 0 |
| pit_ahead_s1 | 29.16 | 2.503 | 0 | False | STUCK | SAFE CLEAR, CAUTION FLOOR_LOST, SAFE OBSTACLE | 5 |
| pit_ahead_s2 | 29.06 | 2.452 | 0 | False | STUCK | SAFE CLEAR, CAUTION FLOOR_LOST, SAFE OBSTACLE | 5 |
| pit_ahead_s3 | 97.71 | 9.669 | 0 | False | STUCK | SAFE CLEAR, CAUTION FLOOR_LOST, SAFE OBSTACLE | 15 |
| pit_ahead_s4 | 29.41 | 2.512 | 0 | False | STUCK | SAFE CLEAR, CAUTION FLOOR_LOST, SAFE OBSTACLE | 5 |
| pit_ahead_s5 | 92.36 | 9.711 | 0 | False | STUCK | SAFE CLEAR, CAUTION FLOOR_LOST, SAFE OBSTACLE | 15 |
| ramp_s1 | 26.21 | 2.821 | 0 | False | CRUISE | SAFE CLEAR, CAUTION TILT | 5 |
| ramp_s2 | 26.06 | 2.735 | 0 | False | CRUISE | SAFE CLEAR, CAUTION TILT | 5 |
| ramp_s3 | 26.21 | 2.722 | 0 | False | CRUISE | SAFE CLEAR, CAUTION TILT | 5 |
| ramp_s4 | 26.21 | 2.753 | 0 | False | CRUISE | SAFE CLEAR, CAUTION TILT | 5 |
| ramp_s5 | 26.11 | 2.742 | 0 | False | CRUISE | SAFE CLEAR, CAUTION TILT | 5 |
| roof_fall_s1 | 22.21 | 1.54 | 0 | False | HOLD | SAFE CLEAR, DANGER ROOF_FALL | 5 |
| roof_fall_s2 | 22.06 | 1.498 | 0 | False | HOLD | SAFE CLEAR, DANGER ROOF_FALL | 5 |
| roof_fall_s3 | 22.06 | 1.503 | 0 | False | HOLD | SAFE CLEAR, DANGER ROOF_FALL | 5 |
| roof_fall_s4 | 22.06 | 1.504 | 0 | False | HOLD | SAFE CLEAR, DANGER ROOF_FALL | 5 |
| roof_fall_s5 | 21.96 | 1.467 | 0 | False | HOLD | SAFE CLEAR, DANGER ROOF_FALL | 5 |
| side_pit_s1 | 36.81 | 5.276 | 0 | False | CRUISE | SAFE CLEAR, CAUTION FLOOR_LOST, SAFE OBSTACLE | 5 |
| side_pit_s2 | 36.58 | 5.167 | 0 | False | CRUISE | SAFE CLEAR, CAUTION FLOOR_LOST, SAFE OBSTACLE | 5 |
| side_pit_s3 | 36.65 | 5.236 | 0 | False | CRUISE | SAFE CLEAR, CAUTION FLOOR_LOST, SAFE OBSTACLE | 5 |
| side_pit_s4 | 36.77 | 5.241 | 0 | False | CRUISE | SAFE CLEAR, CAUTION FLOOR_LOST, SAFE OBSTACLE | 5 |
| side_pit_s5 | 36.6 | 5.222 | 0 | False | CRUISE | SAFE CLEAR, CAUTION FLOOR_LOST, SAFE OBSTACLE | 5 |
