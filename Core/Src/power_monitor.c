/*
 * power_monitor.h
 *
 *  Created on: Jul 8, 2026
 *      Author: Jackson
 */


#include "power_monitor.h"
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
HAL_StatusTypeDef Power_Manager_Init(Power_State_t state, Power_Monitor_t monitor){

	if (state == NULL || monitor == NULL) {
		return HAL_ERROR;
	}
	monitor->voltage_raw = 0.0f;
	monitor->voltage_filtered = 0.0f;
	monitor->voltage_min = 0.0f;
	monitor->voltage_max = 0.0f;

	monitor->state = POWER_OK;

	monitor->warning_count = 0;
	monitor->fault_count = 0;
	return HAL_OK;
}

// Updates voltage data from ADC
HAL_StatusTypeDef Power_Manager_Read(I2C_HandleTypeDef *hadc1, Power_Monitor_t monitor) {

	if (hadc1 == NULL || monitor == NULL) {
		return HAL_ERROR;
	}

	HAL_ADC_Start(&hadc1);
	if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
		monitor->voltage_raw = HAL_ADC_GetValue(&hadc1);
	}
	HAL_ADC_Stop(&hadc1);
	return HAL_OK;
}

// Updates voltage data from ADC
HAL_StatusTypeDef Power_Manager_Update(Power_State_t *state){
	return;
}

