# STM32F446RE MPU6050 IMU

Embedded C project using an STM32F446RE Nucleo board and an MPU6050 accelerometer/gyroscope to estimate orientation and stream real-time telemetry.

The system communicates with the MPU6050 over I2C, calibrates sensor bias, converts raw accelerometer/gyroscope data into physical units, estimates pitch/roll/yaw, and outputs CSV telemetry over UART for logging and visualization.

## Features

* STM32F446RE firmware written in C
* MPU6050 I2C communication
* WHO_AM_I device verification
* Accelerometer and gyroscope raw data reads
* Unit conversion to `g` and `deg/s`
* Startup calibration and bias compensation
* Complementary filter for pitch and roll
* Gyroscope integration for yaw
* UART CSV telemetry output
* Basic system state and fault tracking
* I2C bus recovery routine for improved startup reliability

## System Architecture

The project is organized into modular firmware components:

| Module               | Purpose                                                        |
| -------------------- | -------------------------------------------------------------- |
| `mpu6050.c/.h`       | Sensor initialization, raw reads, unit conversion, calibration |
| `imu_filter.c/.h`    | `dt` calculation and angle estimation                          |
| `telemetry.c/.h`     | UART CSV output                                                |
| `system_health.c/.h` | Runtime state and error tracking                               |
| `main.c`             | Startup sequence and main loop                                 |

## Telemetry Output

The firmware streams IMU data in CSV format:

```text
Time_ms,dt,AX_g,AY_g,AZ_g,GX_dps,GY_dps,GZ_dps,Pitch,Roll,Yaw
```

This allows the data to be viewed in a serial monitor, logged to a CSV file, or visualized with Python/Serial Studio.

## Filtering

Pitch and roll are estimated using a complementary filter:

```c
angle = 0.98f * (gyro_prediction) + 0.02f * (accelerometer_angle);
```

The gyroscope provides fast short-term response, while the accelerometer corrects long-term drift.

Yaw is currently integrated from the Z-axis gyroscope, so yaw drift is expected because the MPU6050 does not include a magnetometer.

## Reliability Improvement

During rapid firmware resets, the MPU6050/I2C bus could occasionally remain in a busy state. I added an I2C bus recovery routine at startup to manually release the bus before sensor initialization, reducing the need for manual power cycling during debugging.

## Skills Demonstrated

* Embedded C
* STM32 HAL
* I2C sensor communication
* UART telemetry
* IMU calibration
* Complementary filtering
* Modular firmware design
* Hardware/software debugging
* Fault-state tracking

## Future Work

* Add fixed sample-rate control
* Add Python or Serial Studio visualization
* Improve yaw handling with deadband or magnetometer support
* Connect orientation estimates to motor control for a stabilization platform or camera gimbal
