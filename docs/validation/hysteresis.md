# Servo Hysteresis & Actuator Characterization

This section documents the measured limitations of the MG90S servo + gearbox + linkage system used by the two-axis gimbal.

The main result is that the actuator/mechanical plant is **not well described by one fixed symmetric deadband**. Near center, the physical response depends on command direction, prior position, gearbox state, static friction, gravity loading, and linkage compliance.

The complete analysis workbook, including raw telemetry for the major V1/V2 tests, is available here:

- [`servo_hysteresis_characterization_v1_v2.xlsx`](spreadsheets/servo_hysteresis_characterization_v1_v2.xlsx)

## Why This Matters

A stabilization controller repeatedly makes small corrections and reverses direction around the setpoint. If the servo/linkage does not reproduce those commands consistently, the controller can experience lost motion, delayed breakaway, overshoot after slack is taken up, or a small limit cycle even when timing and controller arithmetic are correct.

The measured plant is closer to:

```text
servo command
   |
   v
servo internal deadband
+ gearbox backlash
+ linkage compliance
+ static friction
+ directional gravity/load
   |
   v
platform orientation
```

rather than:

```text
|command| < one fixed threshold  -> no movement
|command| > threshold            -> predictable movement
```

## Analysis Convention

Unless otherwise noted:

- telemetry was captured at 20 Hz from a 100 Hz control/sampling schedule
- each settled value is the mean of the final 10 telemetry rows (~0.5 s)
- incremental response is measured relative to the immediately preceding zero-command settled state
- `|response| >= 0.20°` is used only as a descriptive **clear-movement threshold**
- that 0.20° analysis threshold is **not** claimed to be the servo's physical deadband

## V1 Near-Center Baseline

The V1 focused test established that the first clear movement was already asymmetric:

| Axis | Direction | First clear movement observed |
|---|---:|---:|
| Pitch | + | about +0.50° command |
| Pitch | - | about -0.75° command |
| Roll | + | about +1.00° command |
| Roll | - | about -0.25° command, but not consistently repeatable |

The V1 upper-stage horn/control-arm interface also introduced visible compliance and a static lean, so V1 was treated as a functional firmware/control platform rather than a final precision mechanism.

## V2 Broad Near-Center Sweeps

Two independent V2 sweeps again showed direction-dependent breakaway.

| Run | Pitch + | Pitch - | Roll + | Roll - |
|---|---:|---:|---:|---:|
| Run 1 | +1.00° | -0.25° | +0.75° | -0.50° |
| Run 2 | +1.00° | -0.25° | +0.75° | -0.75° |

These are the first clear movements **observed in those specific approach sequences**, not universal actuator thresholds.

The fact that the apparent threshold changes with direction and test history is itself the important result.

## Repeated-Command Repeatability

The same command was repeated three times with a zero-command return between trials. Identical inputs often produced very different incremental motion.

Representative examples:

| Axis / command | Trial 1 | Trial 2 | Trial 3 |
|---|---:|---:|---:|
| Pitch -1.00° | +1.843° | +0.020° | -0.008° |
| Pitch +1.00° | -0.558° | -0.441° | -0.054° |
| Roll +0.50° | -0.795° | -0.369° | -0.070° |
| Roll -1.00° | +0.673° | +0.687° | +0.562° |

The pitch -1.00° result is especially important: the first command moved the mechanism strongly, while the next two nearly identical commands produced almost no incremental motion. A fixed software deadband cannot model that behavior.

## Staircase Hysteresis

The staircase test approached the same command values from different directions without resetting the mechanism between every step. This gives a direct same-command hysteresis measurement.

Largest measured branch gaps:

| Axis | Command | Outbound settled angle | Return settled angle | Branch gap |
|---|---:|---:|---:|---:|
| Pitch | +1.00° | +0.099° | -2.238° | **2.336°** |
| Pitch | +0.50° | +0.065° | -1.882° | **1.947°** |
| Pitch | -0.50° | +0.313° | +2.126° | **1.813°** |
| Roll | +1.50° | -1.222° | -1.704° | **0.482°** |
| Roll | -0.50° | -0.255° | +0.074° | **0.329°** |

Pitch is therefore the dominant hysteresis/lost-motion axis in the current mechanism. Roll hysteresis is smaller but still measurable.

## Long-Hold Breakaway

Small commands were held for 5 s to test whether an initially stuck mechanism would slowly creep and eventually break away.

| Axis | Command | Settled incremental response |
|---|---:|---:|
| Pitch | -0.25° | +0.421° |
| Pitch | +0.75° | +0.017° |
| Roll | +0.50° | -0.273° |
| Roll | -0.25° | +0.362° |

The dominant behavior was **not** slow delayed creep. Some small commands moved early, while pitch +0.75° remained effectively stuck for the full hold. That points more strongly toward static friction, backlash state, and load direction than toward a simple time-dependent creep model.

## Cross-Axis Coupling

Single-axis ±2° commands produced relatively small unintended motion on the orthogonal axis:

| Driven axis | Command | Primary response | Cross-axis response | Coupling |
|---|---:|---:|---:|---:|
| Pitch | +2° | -2.620° | +0.107° | 4.1% |
| Pitch | -2° | +1.933° | -0.090° | 4.7% |
| Roll | +2° | -2.033° | -0.087° | 4.3% |
| Roll | -2° | +1.459° | +0.041° | 2.8% |

Cross-axis coupling is therefore **not the primary limitation**. The larger problem is same-axis hysteresis and directional response asymmetry.

## Raw PWM Characterization

A separate test bypassed degree-based servo offsets and commanded raw pulse-width changes around each calibrated center.

The first clear movement observed in that sweep was approximately:

| Axis | Positive PWM offset | Negative PWM offset |
|---|---:|---:|
| Pitch | +10 µs | -8 µs |
| Roll | +4 µs | -5 µs |

These values are not fixed deadband thresholds. A roll-only repeated reversal test demonstrated the same state dependence at identical raw-PWM commands:

| Roll PWM offset | Trial 1 | Trial 2 | Trial 3 |
|---|---:|---:|---:|
| -10 µs | +0.438° | +0.120° | +0.008° |
| -12 µs | +0.505° | +0.102° | +0.039° |
| -15 µs | +0.720° | +0.978° | +0.736° |
| +10 µs | -0.534° | -0.325° | -0.402° |
| +20 µs | -1.455° | -1.183° | -1.404° |

This confirms that the limitation is not just degree-to-PWM scaling. The raw actuator/mechanical response itself is direction- and history-dependent.

## Open-Loop ±20° Command Accuracy

A larger open-loop test commanded ±20° servo offsets and compared them directly against MPU-measured platform motion.

| Axis | Servo command | Measured platform change | Static gain |
|---|---:|---:|---:|
| Pitch | +20° | -17.426° | 0.871 |
| Pitch | -20° | +15.434° | 0.772 |
| Roll | +20° | -15.764° | 0.788 |
| Roll | -20° | +15.305° | 0.765 |

The servo command therefore does **not** map 1:1 to platform rotation, and pitch is more directionally asymmetric than roll.

The abrupt 20° steps also drove the MPU6050 gyro near its configured ±250°/s limit, so this run is used for **settled static accuracy**, not precise transient velocity or overshoot analysis.

## Intermittent Shaking

The MG90S assembly has also shown intermittent shaking/hunting during ordinary motion and in at least one raw-PWM test. A targeted repeated reversal test did not reproduce sustained shaking every time.

The current evidence therefore supports describing the shaking as **state-dependent actuator/mechanical behavior**, not as a deterministic firmware oscillation. It can occur even when the outer gimbal controller is bypassed and the PWM command is held fixed.

Likely contributors include the MG90S internal position loop, gearbox backlash, compliance, static friction, load direction, and power transients. The tests do not isolate one single cause yet.

## Control Decision Before P Tuning

The characterization supports the following control strategy:

1. Keep the existing **0.5° orientation-error deadband**.
2. Do **not** add a conventional ±3° motor-output deadband that simply suppresses all smaller commands.
3. Tune the closed-loop **P controller first** with `Ki = 0` and `Kd = 0`.
4. If P repeatedly stalls outside the 0.5° error deadband because the actuator cannot break free, test a **direction-specific minimum-actuation / breakaway compensation**.
5. Add integral and derivative action only after the P-only plant behavior is measured.

This preserves small useful corrections while keeping the mechanical nonlinearity visible instead of hiding it behind an arbitrary software threshold.
