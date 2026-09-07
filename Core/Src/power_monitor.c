/*
 * power_monitor.c
 *
 * Created on: Jul 8, 2026
 * Author: Jackson
 */

#include "power_monitor.h"


// Initializes power monitoring structure
HAL_StatusTypeDef Power_Manager_Init(Power_Monitor_t *monitor)
{
    if (monitor == NULL) {
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


// Reads raw voltage data from ADC
HAL_StatusTypeDef Power_Manager_Read(
    ADC_HandleTypeDef *hadc,
    Power_Monitor_t *monitor)
{
    if (hadc == NULL || monitor == NULL) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;

    status = HAL_ADC_Start(hadc);
    if (status != HAL_OK) {
        return status;
    }

    status = HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        HAL_ADC_Stop(hadc);
        return status;
    }

    monitor->voltage_raw = (float)HAL_ADC_GetValue(hadc);

    status = HAL_ADC_Stop(hadc);
    if (status != HAL_OK) {
        return status;
    }

    return HAL_OK;
}


// Updates power state based on voltage data
HAL_StatusTypeDef Power_Manager_Update(Power_Monitor_t *monitor)
{
    if (monitor == NULL) {
        return HAL_ERROR;
    }

    /*
     * Power-state logic can be added here later.
     *
     * Example:
     *
     * if (monitor->voltage_filtered < FAULT_VOLTAGE) {
     *     monitor->state = POWER_FAULT;
     *     monitor->fault_count++;
     * }
     * else if (monitor->voltage_filtered < WARNING_VOLTAGE) {
     *     monitor->state = POWER_WARNING;
     *     monitor->warning_count++;
     * }
     * else {
     *     monitor->state = POWER_OK;
     * }
     */

    return HAL_OK;
}
