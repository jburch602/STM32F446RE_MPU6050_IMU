# STM32F446RE MPU6050 2-Axis Gimbal Controller

Embedded C stabilization project using an STM32F446RE, MPU6050 IMU, and two MG90S servos to control a two-axis gimbal.

The project started as an IMU driver and orientation-estimation system, then expanded into a closed-loop stabilization platform with calibrated servo control, deterministic timing, DMA telemetry, controller safety features, and mechanical characterization.

<p align="center">
  <img src="docs/images/v1_gimbal_prototype.jpg" width="520" alt="V1 two-axis STM32 MPU6050 gimbal prototype">
</p>

<p align="center"><em>V1 prototype used for firmware, timing, control, and mechanical characterization.</em></p>

## Current Status

The V2 mechanical assembly is built and has completed open-loop actuator/mechanical characterization. The firmware currently includes:

- STM32F446RE firmware written in C using STM32 HAL
- MPU6050 communication over I2C
- startup sensor calibration and bias compensation
- complementary-filter pitch and roll estimation
- integrated yaw estimate
- calibrated two-axis servo control
- closed-loop proportional stabilization architecture
- 0.5° control-error deadband
- servo slew-rate limiting
- PI and derivative controller architecture with `Ki = 0` and `Kd = 0`
- conditional anti-windup architecture, currently disabled
- deterministic **100 Hz** control scheduling using TIM6
- **20 Hz** CSV telemetry using UART TX DMA at 460800 baud
- runtime state and error tracking
- I2C bus recovery for reliable startup after rapid resets
- V1/V2 servo, backlash, hysteresis, raw-PWM, and cross-axis characterization

The next control milestone is **P-only closed-loop tuning** on V2. Integral and derivative gains remain disabled until proportional behavior is measured on the characterized plant.

## Control Architecture

```text
MPU6050
   |
   v
Bias Compensation
   |
   v
Complementary Filter
   |
   v
Pitch / Roll Estimate
   |
   v
Control Error
   |--------------------> Raw Error Derivative
   v
0.5° Deadband
   |
   +----> P Term
   |
   +----> I Term
   |
   +--------------------> D Term
              |
              v
          P + I + D
              |
              v
        Output Clamp
              |
              v
        Slew-Rate Limit
              |
              v
       Calibrated Servos
```

`Ki` and `Kd` are currently zero, so active V1 behavior remains proportional control with deadband and slew limiting.

## Real-Time Timing

The original polling implementation ran at roughly 40 Hz. UART optimization, DMA telemetry, and hardware timer scheduling reduced the control period to a deterministic 10 ms.

| Stage | Approx. Control Period | Approx. Rate |
|---|---:|---:|
| Initial implementation | 24–25 ms | ~40 Hz |
| Higher UART baud rate | ~16 ms | ~62 Hz |
| UART TX DMA | ~13 ms | ~77 Hz |
| TIM6 scheduler | **10 ms** | **100 Hz** |

TIM6 only records control ticks in the interrupt callback. I2C reads, filtering, control calculations, telemetry formatting, and DMA submission remain in the main context rather than the ISR.

Telemetry is decimated to 20 Hz so control timing is independent of serial logging.

## Hardware

| Component | Configuration |
|---|---|
| MCU | NUCLEO-F446RE |
| IMU | MPU6050 |
| Pitch servo | MG90S, TIM8 CH2 |
| Roll servo | MG90S, TIM4 CH1 |
| Servo PWM | 50 Hz |
| Control loop | TIM6, 100 Hz |
| Telemetry | USART2 TX DMA, 460800 baud |
| IMU interface | I2C1 |

Servo center values are empirically calibrated in firmware for the current mechanical revision. The tested PWM operating range is approximately **500–2500 µs**; center values are intentionally kept in the servo/gimbal configuration rather than treated as universal 1500 µs constants.

## Mechanical Characterization

V1 and V2 testing show that the MG90S servo + gearbox + linkage system has **direction-dependent, history-dependent hysteresis**, not one clean symmetric deadband.

The strongest V2 results include:

- same-command pitch branch gaps up to **2.336°** in a staircase hysteresis test
- repeated pitch -1.0° commands producing **+1.843°, +0.020°, and -0.008°** incremental responses
- raw-PWM breakaway that changes with direction and prior mechanical state
- approximately **0.77–0.87** platform-angle gain for ±20° servo commands rather than a 1:1 command-to-platform relationship
- relatively low cross-axis coupling of roughly **3–5%**
- intermittent servo/mechanical shaking that can occur even with the outer gimbal controller bypassed

These measurements indicate that gearbox backlash, servo internal behavior, static friction, gravity loading, and linkage compliance currently dominate near-center precision more than cross-axis coupling.

Detailed methodology, raw-data references, and the current control implications are documented in [`docs/validation/hysteresis.md`](docs/validation/hysteresis.md).

## Validation Results

The project is validated with logged telemetry rather than visual observation alone.

| Test | Result |
|---|---|
| Control scheduling | Deterministic 100 Hz control loop with measured `dt = 0.0100 s` |
| Telemetry | 20 Hz UART TX DMA with zero observed telemetry errors in validated runs |
| Deadband | 0.5° deadband reduced in-band pitch/roll command RMS to 0° and suppressed 100% of in-band commands |
| Slew limiter | Aggressive reversals reached the configured 120°/s ceiling on both axes without exceeding it |
| V2 staircase hysteresis | Maximum same-command branch gap measured at **2.336° pitch** and **0.482° roll** |
| Repeatability | Identical small servo commands can produce substantially different physical motion depending on mechanical history |
| Cross-axis coupling | Approximately **3–5%**, lower than same-axis hysteresis effects |
| ±20° static step | Platform motion is approximately **0.77–0.87×** the servo command magnitude |

Detailed controller-validation workbooks are in [`docs/validation`](docs/validation/README.md). The consolidated V1/V2 actuator characterization is in [`docs/validation/hysteresis.md`](docs/validation/hysteresis.md), with raw telemetry in [`servo_hysteresis_characterization_v1_v2.xlsx`](docs/validation/spreadsheets/servo_hysteresis_characterization_v1_v2.xlsx).

## V2 Mechanical Revision

V2 was built to reduce the largest V1 structural uncertainty and provide a cleaner platform for control tuning. The redesign improved the mechanical assembly enough to support repeatable characterization, but the MG90S actuator/linkage system still shows measurable hysteresis and directional asymmetry.

<p align="center">
  <img src="docs/images/v2_gimbal_prototype.jpg" width="520" alt="V2 two-axis STM32 MPU6050 gimbal prototype">
</p>

<p align="center"><em>V2 mechanical assembly after the upper-stage redesign.</em></p>

Compared with V1, the upper stage is substantially more centered and the servo-horn/control-arm interface is much stiffer. A small residual static lean of roughly **1–3°** remains, but visible structural flex has been greatly reduced. This makes the remaining near-center error easier to attribute to the MG90S servos, gearbox backlash, stiction, and load asymmetry rather than gross frame compliance. The residual static bias can be handled separately through mechanical alignment or center calibration; the measured hysteresis cannot be removed by a simple center offset.

Current control decision:

1. retain the existing 0.5° orientation-error deadband
2. keep `Ki = 0` and `Kd = 0`
3. tune proportional gain on V2
4. only add minimum-actuation / breakaway compensation if P-only data shows the servo repeatedly stalls outside the error deadband
5. tune integral and derivative action after proportional behavior is understood

A conventional fixed **3° motor-output deadband is not currently used**, because the measured actuator behavior is not one fixed symmetric threshold.

## Firmware Organization

| Module | Purpose |
|---|---|
| `mpu6050.c/.h` | Sensor initialization, raw reads, unit conversion, calibration |
| `imu_filter.c/.h` | `dt` measurement and complementary-filter angle estimation |
| `servo.c/.h` | PWM servo interface and calibrated pulse/offset control |
| `gimbal.c/.h` | Closed-loop two-axis stabilization logic |
| `telemetry.c/.h` | CSV telemetry and UART DMA transmission |
| `system_health.c/.h` | Runtime state and error tracking |
| `i2c_manager.c/.h` | I2C bus recovery |
| `main.c` | Startup sequence and 100 Hz control scheduler |

## Reliability and Validation

Several implementation details were added specifically from observed failures or measured behavior:

- I2C recovery is executed before I2C peripheral initialization to recover from a stuck bus after rapid firmware resets.
- Servo PWM starts at empirically calibrated center pulse widths to avoid a startup jump to a generic 1500 µs position.
- UART telemetry uses a persistent DMA buffer so the CPU is not blocked for the duration of each serial packet.
- Control scheduling is timer-driven rather than paced by `HAL_Delay()`.
- Measured `dt` is retained in the filter/controller even with deterministic scheduling.
- Missed scheduler periods are not replayed as fake catch-up sensor samples.
- Mechanical step, raw-PWM, and hysteresis testing are logged through the same IMU and telemetry path used during closed-loop operation.

Validated 100 Hz test captures showed:

- `dt = 0.0100 s`
- five control samples per 20 Hz telemetry record
- zero MPU read, `dt`, angle, and telemetry errors during tested runs

## Telemetry

Current CSV telemetry includes IMU data, orientation, controller state, servo commands, and health counters.

Example fields:

```text
Time_ms,dt,
AX_g,AY_g,AZ_g,
GX_dps,GY_dps,GZ_dps,
Pitch,Roll,Yaw,
PitchError_deg,RollError_deg,
PitchCommand_deg,RollCommand_deg,
State,DataValid,
SampleCount,ValidCount,
FailedRead,FailedDt,FailedAngle,FailedTelemetry,
ReadErrPct,TotalErrPct
```

## Skills Demonstrated

- Embedded C
- STM32 HAL and CubeMX
- I2C sensor interfaces
- PWM motor/servo control
- timer-based real-time scheduling
- UART DMA
- IMU calibration and sensor fusion
- closed-loop control architecture
- controller deadband, slew limiting, and anti-windup design
- system health/error instrumentation
- hardware/software debugging
- quantitative mechanical characterization
- open-loop actuator characterization and hysteresis analysis

## Next Steps

1. Tune **P-only** closed-loop stabilization on V2 with `Ki = 0` and `Kd = 0`.
2. Quantify rise time, settling behavior, steady-state error, and near-zero limit cycling.
3. Add direction-specific minimum-actuation / breakaway compensation only if P-only data justifies it.
4. Tune integral and derivative gains after proportional behavior is stable and understood.
5. Re-evaluate the MG90S actuator choice if servo hunting/backlash remains the dominant performance limit.
