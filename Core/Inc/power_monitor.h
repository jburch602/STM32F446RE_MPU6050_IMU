/*
 * power_monitor.h
 *
 *  Created on: Jul 8, 2026
 *      Author: Jackson
 */

#ifndef INC_POWER_MONITOR_H_
#define INC_POWER_MONITOR_H_

#include "stm32f4xx_hal.h"


typedef enum {
	POWER_OK = 0,
	POWER_WARNING = 1,
	POWER_FAULT = 2
} Power_State_t;

typedef struct {
	float voltage_raw;
	float voltage_filtered;
	float voltage_min;
	float voltage_max;

	Power_State_t state;

	uint32_t warning_count;
	uint32_t fault_count;
} Power_Monitor_t;
//Initializes power management
HAL_StatusTypeDef Power_Manager_Init(Power_State_t *state);

// Updates voltage data from ADC
HAL_StatusTypeDef Power_Manager_Read(I2C_HandleTypeDef *hadc1, Power_State_t *state);

// Updates voltage data from ADC
HAL_StatusTypeDef Power_Manager_Update(Power_State_t *state);


#endif /* INC_POWER_MANAGER_H_ */
