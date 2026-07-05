# STM32F446RE_MPU6050_IMU

## I2C Bus Recovery Documentation

### Problem Observed

During development, rapid restarts of the firmware could result in failed initialization of the MPU. Below is the error observed.

    # STATUS: The MPU failed to initialize System=7 HAL_Status=2

That is a `SYS_FAULT` state with a `HAL_BUSY` status during the MPU initialization step.

The issue was resolved by a full power cycle, suggesting the I2C bus or MPU6050 was left in a busy or stuck state.

### Plausible Cause

The system was quickly reset during an I2C transaction. The slave may have still been waiting for clocks or was holding SDA low.

### Solution

To reduce the need for manual power cycling, I added an I2C bus recovery routine on startup.
