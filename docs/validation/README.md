# Validation & Test Data

This directory contains the curated validation artifacts used to verify controller behavior and characterize the gimbal mechanics.

The goal is to preserve the engineering evidence behind firmware and control decisions without turning the repository into a dump of every development log.

## Included Tests

| Test | Main result | Artifact |
|---|---|---|
| Deadband baseline | With no deadband, near-zero command RMS was **0.238° pitch** and **0.212° roll** | [`deadband_baseline_db0.xlsx`](spreadsheets/deadband_baseline_db0.xlsx) |
| 0.5° deadband | 100% of samples inside the ±0.5° measured-angle band produced a 0° servo command | [`deadband_validation_db0p5.xlsx`](spreadsheets/deadband_validation_db0p5.xlsx) |
| 120°/s slew limiter | Aggressive reversals reached the configured **120°/s** ceiling on both axes with no observed violations | [`slew_rate_aggressive_reversal_120dps.xlsx`](spreadsheets/slew_rate_aggressive_reversal_120dps.xlsx) |
| V1/V2 servo hysteresis | Direction-dependent breakaway, same-command branch gaps, repeatability error, raw-PWM state dependence, and non-1:1 servo/platform gain | [`hysteresis.md`](hysteresis.md) |
| Hysteresis workbook | Consolidated V1/V2 analysis with raw telemetry sheets | [`servo_hysteresis_characterization_v1_v2.xlsx`](spreadsheets/servo_hysteresis_characterization_v1_v2.xlsx) |

## Deadband Validation

Inside the ±0.5° measured-angle region:

- pitch command RMS fell from **0.238° to 0.000°**
- roll command RMS fell from **0.212° to 0.000°**
- the 0.5° configuration suppressed **100%** of in-band pitch and roll commands

This verifies that the control-error deadband removes small command chatter near level rather than merely reducing it.

![Deadband comparison](images/deadband_comparison.png)

## Slew-Rate Validation

Measured results from the aggressive reversal test:

- maximum observed pitch command rate: **120.0°/s**
- maximum observed roll command rate: **120.0°/s**
- configured limit: **120°/s**
- near-limit hits: **119 pitch**, **30 roll**
- observed rate-limit violations: **0**
- final 5 s roll peak-to-peak motion: approximately **0.190°**

![Slew-rate validation](images/slew_rate_validation.png)

## Servo / Mechanical Hysteresis

V1 and V2 testing showed that the MG90S + gearbox + linkage system is not accurately modeled by one fixed symmetric deadband.

Key V2 findings include:

- maximum measured pitch staircase branch gap: **2.336°**
- maximum measured roll staircase branch gap: **0.482°**
- identical pitch -1.0° commands produced responses of **+1.843°, +0.020°, and -0.008°** across repeated trials
- raw-PWM response remained direction- and history-dependent
- ±20° servo commands produced only about **0.77–0.87** platform-angle gain
- cross-axis coupling remained relatively low at roughly **3–5%**

The detailed methodology, tables, interpretation, and control implications are documented in [`hysteresis.md`](hysteresis.md).

## Engineering Interpretation

The current controller can generate sub-degree corrections, but the hobby-servo mechanics do not reproduce those commands symmetrically or repeatably around center.

That matters because stabilization repeatedly reverses direction near the setpoint:

```text
small error
→ small servo correction
→ mechanical play / static friction absorbs part of the command
→ error persists
→ mechanism eventually breaks free
→ controller reverses
→ hysteresis is encountered again
```

For this reason, the current control plan is to retain the **0.5° orientation-error deadband**, tune **P-only** behavior next, and add minimum-actuation compensation only if closed-loop data demonstrates that P stalls outside the error deadband.

## Data Notes

- IMU/controller telemetry is logged as CSV over USART2.
- Current deterministic controller timing is 100 Hz with telemetry decimated to 20 Hz.
- Servo-command degrees are not assumed to equal platform degrees because linkage geometry, gravity loading, internal servo behavior, backlash, friction, and compliance affect the transfer ratio.
- Raw telemetry for the principal hysteresis tests is preserved inside the consolidated hysteresis workbook.
