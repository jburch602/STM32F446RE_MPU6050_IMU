# STM32F446RE MPU6050 2-Axis Gimbal Controller

Embedded C stabilization project using an STM32F446RE, MPU6050 IMU, and two MG90S servos to control a two-axis gimbal.

The project started as an IMU driver and orientation-estimation system, then expanded into a closed-loop stabilization platform with calibrated servo control, deterministic timing, DMA telemetry, controller safety features, and mechanical characterization.

<p align="center">
  <img src="docs/images/v1_gimbal_prototype.jpg" width="520" alt="V1 two-axis STM32 MPU6050 gimbal prototype">
</p>

<p align="center"><em>V1 prototype used for firmware, timing, control, and mechanical characterization.</em></p>

## Current Status

V1 is functional and stabilizes pitch and roll. The firmware currently includes:

- STM32F446RE firmware written in C using STM32 HAL
- MPU6050 communication over I2C
- startup sensor calibration and bias compensation
- complementary-filter pitch and roll estimation
- integrated yaw estimate
- calibrated two-axis servo control
- closed-loop proportional stabilization
- 0.5° control deadband
- servo slew-rate limiting
- PI and derivative controller architecture with `Ki = 0` and `Kd = 0`
- conditional anti-windup architecture, currently disabled
- deterministic **100 Hz** control scheduling using TIM6
- **20 Hz** CSV telemetry using UART TX DMA at 460800 baud
- runtime state and error tracking
- I2C bus recovery for reliable startup after rapid resets

The controller is intentionally being kept at P-only behavior on V1 while the mechanical design is characterized. Final PID tuning will be performed after the V2 mechanical redesign.

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

Empirically calibrated servo pulse ranges:

| Axis | Minimum | Center | Maximum |
|---|---:|---:|---:|
| Pitch | 500 µs | 1530 µs | 2500 µs |
| Roll | 500 µs | 1540 µs | 2500 µs |

## Mechanical Characterization

V1 testing exposed limitations that are mechanical rather than purely control-software problems.

Near-center step tests using commands from ±0.25° through ±2.0° showed **direction-dependent hysteresis and breakaway behavior**. Small corrections do not produce a consistent platform response in both directions, which can contribute to near-zero wobble or limit cycling.

The main V1 mechanical limitation is the link between the lower servo and upper stage. The servo horn does not sit flush against the printed control arm, causing:

- approximately 5–10° of static upper-stage lean
- compliance at the horn/control-arm interface
- rocking during direction reversals
- asymmetric loading from gravity
- additional apparent backlash beyond the servo gear train itself

Because of this, V1 is being used as a firmware and system-characterization platform rather than as the final tuning target.

## Validation Results

The project is validated with logged telemetry rather than visual observation alone.

| Test | Result |
|---|---|
| Control scheduling | Deterministic 100 Hz control loop with measured `dt = 0.0100 s` |
| Telemetry | 20 Hz UART TX DMA with zero observed telemetry errors in validated runs |
| Deadband | 0.5° deadband reduced in-band pitch/roll command RMS to 0° and suppressed 100% of in-band commands |
| Slew limiter | Aggressive reversals reached the configured 120°/s ceiling on both axes without exceeding it |
| Mechanical characterization | Direction-dependent near-center hysteresis identified in the V1 servo/linkage assembly |

Detailed workbooks, plots, methodology, and interpretation are available in [`docs/validation`](docs/validation/README.md).

## V2 Mechanical Goals

The next revision will focus on reducing mechanical uncertainty before tuning integral and derivative gains:

- recess the servo horn into a flat locating pocket
- create a rigid, flush horn-to-link interface
- reduce direction-dependent play and compliance
- repeat near-center response testing
- tune `Ki` and `Kd` only after mechanical improvements are validated

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
- Mechanical step testing is logged through the same IMU and telemetry path used during closed-loop operation.

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

## Next Steps

1. Redesign the V1 inter-axis servo/control-arm interface.
2. Build and validate the V2 mechanical assembly.
3. Repeat backlash/hysteresis and step-response measurements.
4. Tune P, I, and D gains on the mechanically improved platform.
5. Compare V1 and V2 stabilization performance quantitatively.
