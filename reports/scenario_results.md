# Scenario results

Generated 2026-11-13 19:15 by `scenarios/run_all.sh` (noise/shift seeds n1s0, n0s0, n1s1, n2s2, n3s3, n4s4, n5s5; headless fast mode).

| scenario | n1s0 | n0s0 | n1s1 | n2s2 | n3s3 | n4s4 | n5s5 | pass rate |
|---|---|---|---|---|---|---|---|---|
| gas_ahead | PASS | PASS | PASS | PASS | PASS | PASS | PASS | 7/7 |
| gas_dropout | PASS | PASS | PASS | PASS | PASS | PASS | PASS | 7/7 |
| mine | PASS | PASS | PASS | PASS | PASS | PASS | PASS | 7/7 |
| obstacles | PASS | PASS | PASS | PASS | PASS | PASS | PASS | 7/7 |
| pit_ahead | PASS | FAIL | PASS | PASS | FAIL | PASS | FAIL | 4/7 |
| ramp | PASS | PASS | PASS | PASS | PASS | PASS | PASS | 7/7 |
| roof_fall | PASS | PASS | PASS | PASS | PASS | PASS | PASS | 7/7 |
| side_pit | PASS | PASS | PASS | PASS | PASS | PASS | PASS | 7/7 |

**Overall: 53/56 runs passed.**

## Failed checks

- `pit_ahead_n0s0` (ok): final note contains 'route blocked' (STUCK: no clear direction found)
- `pit_ahead_n3s3` (ok): final note contains 'route blocked' (STUCK: no clear direction found)
- `pit_ahead_n5s5` (ok): final note contains 'route blocked' (STUCK: no clear direction found)

## Per-run details

| run | sim s | path m | collisions | fell | final nav | hazards seen | images |
|---|---|---|---|---|---|---|---|
| gas_ahead_n1s0 | 38.91 | 3.134 | 0 | False | HOLD | SAFE CLEAR, CAUTION GAS_ELEVATED, DANGER GAS_BUILDUP | 10 |
| gas_ahead_n0s0 | 39.06 | 3.124 | 0 | False | HOLD | SAFE CLEAR, CAUTION GAS_ELEVATED, DANGER GAS_BUILDUP | 10 |
| gas_ahead_n1s1 | 38.91 | 3.161 | 0 | False | HOLD | SAFE CLEAR, CAUTION GAS_ELEVATED, DANGER GAS_BUILDUP | 10 |
| gas_ahead_n2s2 | 38.91 | 3.089 | 0 | False | HOLD | SAFE CLEAR, CAUTION GAS_ELEVATED, DANGER GAS_BUILDUP | 10 |
| gas_ahead_n3s3 | 39.01 | 3.158 | 0 | False | HOLD | SAFE CLEAR, CAUTION GAS_ELEVATED, DANGER GAS_BUILDUP | 10 |
| gas_ahead_n4s4 | 39.01 | 3.162 | 0 | False | HOLD | SAFE CLEAR, CAUTION GAS_ELEVATED, DANGER GAS_BUILDUP | 10 |
| gas_ahead_n5s5 | 38.91 | 3.129 | 0 | False | HOLD | SAFE CLEAR, CAUTION GAS_ELEVATED, DANGER GAS_BUILDUP | 10 |
| gas_dropout_n1s0 | 16.62 | 2.741 | 0 | False | CRUISE | SAFE CLEAR, CAUTION STALE_DATA | 0 |
| gas_dropout_n0s0 | 16.82 | 2.739 | 0 | False | CRUISE | SAFE CLEAR, CAUTION STALE_DATA | 0 |
| gas_dropout_n1s1 | 16.76 | 2.779 | 0 | False | CRUISE | SAFE CLEAR, CAUTION STALE_DATA | 0 |
| gas_dropout_n2s2 | 16.56 | 2.697 | 0 | False | CRUISE | SAFE CLEAR, CAUTION STALE_DATA | 0 |
| gas_dropout_n3s3 | 16.78 | 2.77 | 0 | False | CRUISE | SAFE CLEAR, CAUTION STALE_DATA | 0 |
| gas_dropout_n4s4 | 16.78 | 2.771 | 0 | False | CRUISE | SAFE CLEAR, CAUTION STALE_DATA | 0 |
| gas_dropout_n5s5 | 16.6 | 2.731 | 0 | False | CRUISE | SAFE CLEAR, CAUTION STALE_DATA | 0 |
| mine_n1s0 | 89.96 | 12.353 | 0 | False | HOLD | SAFE CLEAR, CAUTION FLOOR_LOST, SAFE OBSTACLE, CAUTION GAS_ELEVATED, DANGER GAS_BUILDUP | 15 |
| mine_n0s0 | 91.16 | 12.392 | 0 | False | HOLD | SAFE CLEAR, CAUTION FLOOR_LOST, SAFE OBSTACLE, CAUTION GAS_ELEVATED, DANGER GAS_BUILDUP | 15 |
| mine_n1s1 | 91.41 | 12.424 | 0 | False | HOLD | SAFE CLEAR, SAFE OBSTACLE, CAUTION FLOOR_LOST, CAUTION GAS_ELEVATED, DANGER GAS_BUILDUP | 15 |
| mine_n2s2 | 90.51 | 12.317 | 0 | False | HOLD | SAFE CLEAR, CAUTION FLOOR_LOST, SAFE OBSTACLE, CAUTION GAS_ELEVATED, DANGER GAS_BUILDUP | 15 |
| mine_n3s3 | 90.86 | 12.384 | 0 | False | HOLD | SAFE CLEAR, SAFE OBSTACLE, CAUTION FLOOR_LOST, CAUTION GAS_ELEVATED, DANGER GAS_BUILDUP | 15 |
| mine_n4s4 | 90.26 | 12.38 | 0 | False | HOLD | SAFE CLEAR, CAUTION FLOOR_LOST, SAFE OBSTACLE, CAUTION GAS_ELEVATED, DANGER GAS_BUILDUP | 15 |
| mine_n5s5 | 90.51 | 12.336 | 0 | False | HOLD | SAFE CLEAR, SAFE OBSTACLE, CAUTION FLOOR_LOST, CAUTION GAS_ELEVATED, DANGER GAS_BUILDUP | 15 |
| obstacles_n1s0 | 28.42 | 6.59 | 0 | False | CRUISE | SAFE CLEAR | 0 |
| obstacles_n0s0 | 28.6 | 6.583 | 0 | False | CRUISE | SAFE CLEAR | 0 |
| obstacles_n1s1 | 28.52 | 6.626 | 0 | False | CRUISE | SAFE CLEAR | 0 |
| obstacles_n2s2 | 27.72 | 6.531 | 0 | False | CRUISE | SAFE CLEAR | 0 |
| obstacles_n3s3 | 28.23 | 6.626 | 0 | False | CRUISE | SAFE CLEAR, SAFE OBSTACLE | 0 |
| obstacles_n4s4 | 28.17 | 6.622 | 0 | False | CRUISE | SAFE CLEAR | 0 |
| obstacles_n5s5 | 27.82 | 6.565 | 0 | False | CRUISE | SAFE CLEAR | 0 |
| pit_ahead_n1s0 | 28.76 | 2.479 | 0 | False | STUCK | SAFE CLEAR, CAUTION FLOOR_LOST, SAFE OBSTACLE | 5 |
| pit_ahead_n0s0 | 39.81 | 3.573 | 0 | False | STUCK | SAFE CLEAR, CAUTION FLOOR_LOST, SAFE OBSTACLE | 5 |
| pit_ahead_n1s1 | 29.16 | 2.503 | 0 | False | STUCK | SAFE CLEAR, CAUTION FLOOR_LOST, SAFE OBSTACLE | 5 |
| pit_ahead_n2s2 | 29.06 | 2.452 | 0 | False | STUCK | SAFE CLEAR, CAUTION FLOOR_LOST, SAFE OBSTACLE | 5 |
| pit_ahead_n3s3 | 39.66 | 3.606 | 0 | False | STUCK | SAFE CLEAR, CAUTION FLOOR_LOST, SAFE OBSTACLE | 5 |
| pit_ahead_n4s4 | 29.41 | 2.512 | 0 | False | STUCK | SAFE CLEAR, CAUTION FLOOR_LOST, SAFE OBSTACLE | 5 |
| pit_ahead_n5s5 | 39.51 | 3.599 | 0 | False | STUCK | SAFE CLEAR, CAUTION FLOOR_LOST, SAFE OBSTACLE | 5 |
| ramp_n1s0 | 26.16 | 2.771 | 0 | False | CRUISE | SAFE CLEAR, CAUTION TILT | 5 |
| ramp_n0s0 | 26.21 | 2.692 | 0 | False | CRUISE | SAFE CLEAR, CAUTION TILT | 5 |
| ramp_n1s1 | 26.21 | 2.821 | 0 | False | CRUISE | SAFE CLEAR, CAUTION TILT | 5 |
| ramp_n2s2 | 26.06 | 2.735 | 0 | False | CRUISE | SAFE CLEAR, CAUTION TILT | 5 |
| ramp_n3s3 | 26.21 | 2.722 | 0 | False | CRUISE | SAFE CLEAR, CAUTION TILT | 5 |
| ramp_n4s4 | 26.21 | 2.753 | 0 | False | CRUISE | SAFE CLEAR, CAUTION TILT | 5 |
| ramp_n5s5 | 26.11 | 2.742 | 0 | False | CRUISE | SAFE CLEAR, CAUTION TILT | 5 |
| roof_fall_n1s0 | 21.96 | 1.471 | 0 | False | HOLD | SAFE CLEAR, DANGER ROOF_FALL | 5 |
| roof_fall_n0s0 | 21.96 | 1.44 | 0 | False | HOLD | SAFE CLEAR, DANGER ROOF_FALL | 5 |
| roof_fall_n1s1 | 22.21 | 1.54 | 0 | False | HOLD | SAFE CLEAR, DANGER ROOF_FALL | 5 |
| roof_fall_n2s2 | 22.06 | 1.498 | 0 | False | HOLD | SAFE CLEAR, DANGER ROOF_FALL | 5 |
| roof_fall_n3s3 | 22.06 | 1.503 | 0 | False | HOLD | SAFE CLEAR, DANGER ROOF_FALL | 5 |
| roof_fall_n4s4 | 22.06 | 1.504 | 0 | False | HOLD | SAFE CLEAR, DANGER ROOF_FALL | 5 |
| roof_fall_n5s5 | 21.96 | 1.467 | 0 | False | HOLD | SAFE CLEAR, DANGER ROOF_FALL | 5 |
| side_pit_n1s0 | 37.25 | 5.221 | 0 | False | CRUISE | SAFE CLEAR, CAUTION FLOOR_LOST, SAFE OBSTACLE | 5 |
| side_pit_n0s0 | 36.91 | 5.194 | 0 | False | CRUISE | SAFE CLEAR, CAUTION FLOOR_LOST, SAFE OBSTACLE | 5 |
| side_pit_n1s1 | 36.81 | 5.276 | 0 | False | CRUISE | SAFE CLEAR, CAUTION FLOOR_LOST, SAFE OBSTACLE | 5 |
| side_pit_n2s2 | 36.58 | 5.167 | 0 | False | CRUISE | SAFE CLEAR, CAUTION FLOOR_LOST, SAFE OBSTACLE | 5 |
| side_pit_n3s3 | 36.65 | 5.236 | 0 | False | CRUISE | SAFE CLEAR, CAUTION FLOOR_LOST, SAFE OBSTACLE | 5 |
| side_pit_n4s4 | 36.77 | 5.241 | 0 | False | CRUISE | SAFE CLEAR, CAUTION FLOOR_LOST, SAFE OBSTACLE | 5 |
| side_pit_n5s5 | 36.6 | 5.222 | 0 | False | CRUISE | SAFE CLEAR, CAUTION FLOOR_LOST, SAFE OBSTACLE | 5 |
