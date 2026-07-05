/*
 * i2c_manager.h
 *
 *  Created on: Jul 5, 2026
 *      Author: Jackson
 */

#ifndef INC_I2C_MANAGER_H_
#define INC_I2C_MANAGER_H_

#include "stm32f4xx_hal.h"

// Attempts to recover a stuck I2C1 bus before normal HAL I2C initialization
void I2C_Manager_RecoverI2C1Bus(void);

#endif /* INC_I2C_MANAGER_H_ */
