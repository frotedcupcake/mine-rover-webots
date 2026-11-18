# Scenario results

Generated 2026-11-14 03:26 by `scenarios/run_all.sh` (noise/shift seeds n1s0, n0s0, n1s1, n2s2, n3s3, n4s4, n5s5; headless fast mode).

| scenario | n1s0 | n0s0 | n1s1 | n2s2 | n3s3 | n4s4 | n5s5 | pass rate |
|---|---|---|---|---|---|---|---|---|
| battery_reserve | PASS | PASS | PASS | PASS | PASS | PASS | PASS | 7/7 |
| blocked_ahead | PASS | PASS | PASS | PASS | PASS | PASS | PASS | 7/7 |
| blocked_both | PASS | PASS | PASS | PASS | PASS | PASS | PASS | 7/7 |
| danger_return | PASS | PASS | PASS | PASS | PASS | PASS | PASS | 7/7 |
| gas_ahead | PASS | PASS | PASS | PASS | PASS | PASS | PASS | 7/7 |
| gas_dropout | PASS | PASS | PASS | PASS | PASS | PASS | PASS | 7/7 |
| gas_return | PASS | PASS | PASS | PASS | PASS | PASS | PASS | 7/7 |
| mine | PASS | PASS | PASS | PASS | PASS | PASS | PASS | 7/7 |
| mission_timer | PASS | PASS | PASS | PASS | PASS | PASS | PASS | 7/7 |
| obstacles | PASS | PASS | PASS | PASS | PASS | PASS | PASS | 7/7 |
| pit_ahead | PASS | PASS | PASS | PASS | PASS | PASS | PASS | 7/7 |
| ramp | PASS | PASS | PASS | PASS | PASS | PASS | PASS | 7/7 |
| ramp_gentle | PASS | PASS | PASS | PASS | PASS | PASS | PASS | 7/7 |
| roof_fall | PASS | PASS | PASS | PASS | PASS | PASS | PASS | 7/7 |
| side_pit | PASS | PASS | PASS | PASS | PASS | PASS | PASS | 7/7 |

**Overall: 105/105 runs passed.**

## Failed checks

None.

## Per-run details

| run | sim s | path m | collisions | fell | final nav | hazards seen | images |
|---|---|---|---|---|---|---|---|
| battery_reserve_n1s0 | 112.41 | 19.443 | 0 | False | HOME | SAFE CLEAR, CAUTION FLOOR_LOST, SAFE OBSTACLE | 10 |
| battery_reserve_n0s0 | 112.41 | 19.096 | 0 | False | HOME | SAFE CLEAR, CAUTION FLOOR_LOST, SAFE OBSTACLE | 10 |
| battery_reserve_n1s1 | 115.76 | 19.981 | 0 | False | HOME | SAFE CLEAR, SAFE OBSTACLE, CAUTION FLOOR_LOST | 10 |
| battery_reserve_n2s2 | 112.81 | 19.318 | 0 | False | HOME | SAFE CLEAR, CAUTION FLOOR_LOST, SAFE OBSTACLE | 10 |
| battery_reserve_n3s3 | 114.81 | 19.739 | 0 | False | HOME | SAFE CLEAR, SAFE OBSTACLE, CAUTION FLOOR_LOST | 10 |
| battery_reserve_n4s4 | 113.06 | 19.489 | 0 | False | HOME | SAFE CLEAR, CAUTION FLOOR_LOST, SAFE OBSTACLE | 10 |
| battery_reserve_n5s5 | 112.91 | 19.501 | 0 | False | HOME | SAFE CLEAR, SAFE OBSTACLE, CAUTION FLOOR_LOST | 10 |
| blocked_ahead_n1s0 | 40.01 | 7.073 | 0 | False | HOME | SAFE CLEAR, SAFE OBSTACLE | 0 |
| blocked_ahead_n0s0 | 40.41 | 7.04 | 0 | False | HOME | SAFE CLEAR, SAFE OBSTACLE | 0 |
| blocked_ahead_n1s1 | 39.91 | 7.045 | 0 | False | HOME | SAFE CLEAR, SAFE OBSTACLE | 0 |
| blocked_ahead_n2s2 | 39.96 | 6.993 | 0 | False | HOME | SAFE CLEAR, SAFE OBSTACLE | 0 |
| blocked_ahead_n3s3 | 40.01 | 7.06 | 0 | False | HOME | SAFE CLEAR, SAFE OBSTACLE | 0 |
| blocked_ahead_n4s4 | 40.16 | 7.07 | 0 | False | HOME | SAFE CLEAR, SAFE OBSTACLE | 0 |
| blocked_ahead_n5s5 | 39.76 | 7.024 | 0 | False | HOME | SAFE CLEAR, SAFE OBSTACLE | 0 |
| blocked_both_n1s0 | 44.11 | 5.863 | 0 | False | STUCK | SAFE CLEAR, SAFE OBSTACLE | 0 |
| blocked_both_n0s0 | 44.61 | 5.801 | 0 | False | STUCK | SAFE CLEAR, SAFE OBSTACLE | 0 |
| blocked_both_n1s1 | 46.21 | 5.844 | 0 | False | STUCK | SAFE CLEAR, SAFE OBSTACLE | 0 |
| blocked_both_n2s2 | 44.21 | 5.758 | 0 | False | STUCK | SAFE CLEAR, SAFE OBSTACLE | 0 |
| blocked_both_n3s3 | 44.31 | 5.833 | 0 | False | STUCK | SAFE CLEAR, SAFE OBSTACLE | 0 |
| blocked_both_n4s4 | 44.41 | 5.838 | 0 | False | STUCK | SAFE CLEAR, SAFE OBSTACLE | 0 |
| blocked_both_n5s5 | 44.01 | 5.785 | 0 | False | STUCK | SAFE CLEAR, SAFE OBSTACLE | 0 |
| danger_return_n1s0 | 106.76 | 13.298 | 0 | False | HOME | SAFE CLEAR, CAUTION GAS_ELEVATED, DANGER GAS_BUILDUP | 10 |
| danger_return_n0s0 | 107.76 | 13.313 | 0 | False | HOME | SAFE CLEAR, CAUTION GAS_ELEVATED, DANGER GAS_BUILDUP | 10 |
| danger_return_n1s1 | 107.21 | 13.373 | 0 | False | HOME | SAFE CLEAR, CAUTION GAS_ELEVATED, DANGER GAS_BUILDUP | 10 |
| danger_return_n2s2 | 107.01 | 13.285 | 0 | False | HOME | SAFE CLEAR, CAUTION GAS_ELEVATED, DANGER GAS_BUILDUP | 10 |
| danger_return_n3s3 | 106.91 | 13.317 | 0 | False | HOME | SAFE CLEAR, CAUTION GAS_ELEVATED, DANGER GAS_BUILDUP | 10 |
| danger_return_n4s4 | 106.91 | 13.33 | 0 | False | HOME | SAFE CLEAR, CAUTION GAS_ELEVATED, DANGER GAS_BUILDUP | 10 |
| danger_return_n5s5 | 106.76 | 13.287 | 0 | False | HOME | SAFE CLEAR, CAUTION GAS_ELEVATED, DANGER GAS_BUILDUP | 10 |
| gas_ahead_n1s0 | 70.46 | 3.152 | 0 | False | HOME | SAFE CLEAR, CAUTION GAS_ELEVATED, DANGER GAS_BUILDUP | 10 |
| gas_ahead_n0s0 | 70.66 | 3.136 | 0 | False | HOME | SAFE CLEAR, CAUTION GAS_ELEVATED, DANGER GAS_BUILDUP | 10 |
| gas_ahead_n1s1 | 70.46 | 3.179 | 0 | False | HOME | SAFE CLEAR, CAUTION GAS_ELEVATED, DANGER GAS_BUILDUP | 10 |
| gas_ahead_n2s2 | 70.41 | 3.1 | 0 | False | HOME | SAFE CLEAR, CAUTION GAS_ELEVATED, DANGER GAS_BUILDUP | 10 |
| gas_ahead_n3s3 | 70.51 | 3.169 | 0 | False | HOME | SAFE CLEAR, CAUTION GAS_ELEVATED, DANGER GAS_BUILDUP | 10 |
| gas_ahead_n4s4 | 70.51 | 3.172 | 0 | False | HOME | SAFE CLEAR, CAUTION GAS_ELEVATED, DANGER GAS_BUILDUP | 10 |
| gas_ahead_n5s5 | 70.41 | 3.14 | 0 | False | HOME | SAFE CLEAR, CAUTION GAS_ELEVATED, DANGER GAS_BUILDUP | 10 |
| gas_dropout_n1s0 | 16.62 | 2.741 | 0 | False | CRUISE | SAFE CLEAR, CAUTION STALE_DATA | 0 |
| gas_dropout_n0s0 | 16.82 | 2.739 | 0 | False | CRUISE | SAFE CLEAR, CAUTION STALE_DATA | 0 |
| gas_dropout_n1s1 | 16.76 | 2.779 | 0 | False | CRUISE | SAFE CLEAR, CAUTION STALE_DATA | 0 |
| gas_dropout_n2s2 | 16.56 | 2.697 | 0 | False | CRUISE | SAFE CLEAR, CAUTION STALE_DATA | 0 |
| gas_dropout_n3s3 | 16.78 | 2.77 | 0 | False | CRUISE | SAFE CLEAR, CAUTION STALE_DATA | 0 |
| gas_dropout_n4s4 | 16.78 | 2.771 | 0 | False | CRUISE | SAFE CLEAR, CAUTION STALE_DATA | 0 |
| gas_dropout_n5s5 | 16.6 | 2.731 | 0 | False | CRUISE | SAFE CLEAR, CAUTION STALE_DATA | 0 |
| gas_return_n1s0 | 170.41 | 13.294 | 0 | False | HOME | SAFE CLEAR, CAUTION GAS_ELEVATED, DANGER GAS_BUILDUP | 25 |
| gas_return_n0s0 | 171.41 | 13.308 | 0 | False | HOME | SAFE CLEAR, CAUTION GAS_ELEVATED, DANGER GAS_BUILDUP | 25 |
| gas_return_n1s1 | 170.91 | 13.367 | 0 | False | HOME | SAFE CLEAR, CAUTION GAS_ELEVATED, DANGER GAS_BUILDUP | 25 |
| gas_return_n2s2 | 170.76 | 13.279 | 0 | False | HOME | SAFE CLEAR, CAUTION GAS_ELEVATED, DANGER GAS_BUILDUP | 25 |
| gas_return_n3s3 | 170.61 | 13.325 | 0 | False | HOME | SAFE CLEAR, CAUTION GAS_ELEVATED, DANGER GAS_BUILDUP | 25 |
| gas_return_n4s4 | 170.61 | 13.324 | 0 | False | HOME | SAFE CLEAR, CAUTION GAS_ELEVATED, DANGER GAS_BUILDUP | 25 |
| gas_return_n5s5 | 170.46 | 13.281 | 0 | False | HOME | SAFE CLEAR, CAUTION GAS_ELEVATED, DANGER GAS_BUILDUP | 25 |
| mine_n1s0 | 172.41 | 21.646 | 0 | False | HOME | SAFE CLEAR, CAUTION FLOOR_LOST, SAFE OBSTACLE, CAUTION GAS_ELEVATED, DANGER GAS_BUILDUP | 20 |
| mine_n0s0 | 174.26 | 21.65 | 0 | False | HOME | SAFE CLEAR, CAUTION FLOOR_LOST, SAFE OBSTACLE, CAUTION GAS_ELEVATED, DANGER GAS_BUILDUP | 20 |
| mine_n1s1 | 174.36 | 21.743 | 0 | False | HOME | SAFE CLEAR, SAFE OBSTACLE, CAUTION FLOOR_LOST, CAUTION GAS_ELEVATED, DANGER GAS_BUILDUP | 20 |
| mine_n2s2 | 173.81 | 21.565 | 0 | False | HOME | SAFE CLEAR, CAUTION FLOOR_LOST, SAFE OBSTACLE, CAUTION GAS_ELEVATED, DANGER GAS_BUILDUP | 20 |
| mine_n3s3 | 174.26 | 21.788 | 0 | False | HOME | SAFE CLEAR, SAFE OBSTACLE, CAUTION FLOOR_LOST, CAUTION GAS_ELEVATED, DANGER GAS_BUILDUP | 20 |
| mine_n4s4 | 173.31 | 21.67 | 0 | False | HOME | SAFE CLEAR, CAUTION FLOOR_LOST, SAFE OBSTACLE, CAUTION GAS_ELEVATED, DANGER GAS_BUILDUP | 20 |
| mine_n5s5 | 173.41 | 21.678 | 0 | False | HOME | SAFE CLEAR, SAFE OBSTACLE, CAUTION FLOOR_LOST, CAUTION GAS_ELEVATED, DANGER GAS_BUILDUP | 20 |
| mission_timer_n1s0 | 46.76 | 11.199 | 0 | False | HOME | SAFE CLEAR | 0 |
| mission_timer_n0s0 | 46.76 | 10.887 | 0 | False | HOME | SAFE CLEAR | 0 |
| mission_timer_n1s1 | 46.76 | 11.116 | 0 | False | HOME | SAFE CLEAR | 0 |
| mission_timer_n2s2 | 46.81 | 11.091 | 0 | False | HOME | SAFE CLEAR | 0 |
| mission_timer_n3s3 | 46.56 | 11.092 | 0 | False | HOME | SAFE CLEAR | 0 |
| mission_timer_n4s4 | 46.51 | 11.081 | 0 | False | HOME | SAFE CLEAR | 0 |
| mission_timer_n5s5 | 46.96 | 11.175 | 0 | False | HOME | SAFE CLEAR | 0 |
| obstacles_n1s0 | 28.42 | 6.59 | 0 | False | CRUISE | SAFE CLEAR | 0 |
| obstacles_n0s0 | 28.6 | 6.583 | 0 | False | CRUISE | SAFE CLEAR | 0 |
| obstacles_n1s1 | 28.52 | 6.626 | 0 | False | CRUISE | SAFE CLEAR | 0 |
| obstacles_n2s2 | 27.72 | 6.531 | 0 | False | CRUISE | SAFE CLEAR | 0 |
| obstacles_n3s3 | 28.23 | 6.626 | 0 | False | CRUISE | SAFE CLEAR, SAFE OBSTACLE | 0 |
| obstacles_n4s4 | 28.17 | 6.622 | 0 | False | CRUISE | SAFE CLEAR | 0 |
| obstacles_n5s5 | 27.82 | 6.565 | 0 | False | CRUISE | SAFE CLEAR | 0 |
| pit_ahead_n1s0 | 32.32 | 2.686 | 0 | False | HOME | SAFE CLEAR, CAUTION FLOOR_LOST, SAFE OBSTACLE | 5 |
| pit_ahead_n0s0 | 25.31 | 1.871 | 0 | False | HOME | SAFE CLEAR, CAUTION FLOOR_LOST | 5 |
| pit_ahead_n1s1 | 32.71 | 2.709 | 0 | False | HOME | SAFE CLEAR, CAUTION FLOOR_LOST, SAFE OBSTACLE | 5 |
| pit_ahead_n2s2 | 32.51 | 2.639 | 0 | False | HOME | SAFE CLEAR, CAUTION FLOOR_LOST, SAFE OBSTACLE | 5 |
| pit_ahead_n3s3 | 25.31 | 1.902 | 0 | False | HOME | SAFE CLEAR, CAUTION FLOOR_LOST | 5 |
| pit_ahead_n4s4 | 32.91 | 2.711 | 0 | False | HOME | SAFE CLEAR, CAUTION FLOOR_LOST, SAFE OBSTACLE | 5 |
| pit_ahead_n5s5 | 25.01 | 1.865 | 0 | False | HOME | SAFE CLEAR, CAUTION FLOOR_LOST | 5 |
| ramp_n1s0 | 25.71 | 2.021 | 0 | False | HOME | SAFE CLEAR, CAUTION TILT | 5 |
| ramp_n0s0 | 25.81 | 2.005 | 0 | False | HOME | SAFE CLEAR, CAUTION TILT | 5 |
| ramp_n1s1 | 25.76 | 2.049 | 0 | False | HOME | SAFE CLEAR, CAUTION TILT | 5 |
| ramp_n2s2 | 25.56 | 1.97 | 0 | False | HOME | SAFE CLEAR, CAUTION TILT | 5 |
| ramp_n3s3 | 25.76 | 2.033 | 0 | False | HOME | SAFE CLEAR, CAUTION TILT | 5 |
| ramp_n4s4 | 25.76 | 2.034 | 0 | False | HOME | SAFE CLEAR, CAUTION TILT | 5 |
| ramp_n5s5 | 25.66 | 2.003 | 0 | False | HOME | SAFE CLEAR, CAUTION TILT | 5 |
| ramp_gentle_n1s0 | 22.9 | 5.742 | 0 | False | CRUISE | SAFE CLEAR | 0 |
| ramp_gentle_n0s0 | 23.36 | 5.741 | 0 | False | CRUISE | SAFE CLEAR | 0 |
| ramp_gentle_n1s1 | 23.34 | 5.781 | 0 | False | CRUISE | SAFE CLEAR | 0 |
| ramp_gentle_n2s2 | 23.31 | 5.699 | 0 | False | CRUISE | SAFE CLEAR | 0 |
| ramp_gentle_n3s3 | 23.56 | 5.772 | 0 | False | CRUISE | SAFE CLEAR | 0 |
| ramp_gentle_n4s4 | 23.17 | 5.769 | 0 | False | CRUISE | SAFE CLEAR | 0 |
| ramp_gentle_n5s5 | 22.92 | 5.731 | 0 | False | CRUISE | SAFE CLEAR | 0 |
| roof_fall_n1s0 | 53.46 | 1.489 | 0 | False | HOME | SAFE CLEAR, DANGER ROOF_FALL, SAFE OBSTACLE | 5 |
| roof_fall_n0s0 | 53.51 | 1.452 | 0 | False | HOME | SAFE CLEAR, DANGER ROOF_FALL, SAFE OBSTACLE | 5 |
| roof_fall_n1s1 | 53.71 | 1.558 | 0 | False | HOME | SAFE CLEAR, DANGER ROOF_FALL, SAFE OBSTACLE | 5 |
| roof_fall_n2s2 | 53.61 | 1.508 | 0 | False | HOME | SAFE CLEAR, DANGER ROOF_FALL, SAFE OBSTACLE | 5 |
| roof_fall_n3s3 | 53.61 | 1.514 | 0 | False | HOME | SAFE CLEAR, DANGER ROOF_FALL, SAFE OBSTACLE | 5 |
| roof_fall_n4s4 | 53.61 | 1.515 | 0 | False | HOME | SAFE CLEAR, DANGER ROOF_FALL, SAFE OBSTACLE | 5 |
| roof_fall_n5s5 | 53.61 | 1.481 | 0 | False | HOME | SAFE CLEAR, DANGER ROOF_FALL, SAFE OBSTACLE | 5 |
| side_pit_n1s0 | 37.25 | 5.221 | 0 | False | CRUISE | SAFE CLEAR, CAUTION FLOOR_LOST, SAFE OBSTACLE | 5 |
| side_pit_n0s0 | 36.91 | 5.194 | 0 | False | CRUISE | SAFE CLEAR, CAUTION FLOOR_LOST, SAFE OBSTACLE | 5 |
| side_pit_n1s1 | 36.81 | 5.276 | 0 | False | CRUISE | SAFE CLEAR, CAUTION FLOOR_LOST, SAFE OBSTACLE | 5 |
| side_pit_n2s2 | 36.58 | 5.167 | 0 | False | CRUISE | SAFE CLEAR, CAUTION FLOOR_LOST, SAFE OBSTACLE | 5 |
| side_pit_n3s3 | 36.65 | 5.236 | 0 | False | CRUISE | SAFE CLEAR, CAUTION FLOOR_LOST, SAFE OBSTACLE | 5 |
| side_pit_n4s4 | 36.77 | 5.241 | 0 | False | CRUISE | SAFE CLEAR, CAUTION FLOOR_LOST, SAFE OBSTACLE | 5 |
| side_pit_n5s5 | 36.6 | 5.222 | 0 | False | CRUISE | SAFE CLEAR, CAUTION FLOOR_LOST, SAFE OBSTACLE | 5 |
