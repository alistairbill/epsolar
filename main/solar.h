#pragma once

#include <stdint.h>

#include "esp_err.h"
#include "modbus.h"

enum {
    EPSOLAR_VALID_ARRAY = 1U << 0,
    EPSOLAR_VALID_LOAD = 1U << 1,
    EPSOLAR_VALID_TEMPERATURES = 1U << 2,
    EPSOLAR_VALID_BATTERY_LEVEL = 1U << 3,
    EPSOLAR_VALID_STATUS = 1U << 4,
    EPSOLAR_VALID_BATTERY_ELECTRICAL = 1U << 5,
};

typedef struct {
    uint32_t valid;
    uint16_t array_voltage_cV;
    uint16_t array_current_cA;
    uint32_t array_power_cW;
    uint16_t load_voltage_cV;
    uint16_t load_current_cA;
    uint32_t load_power_cW;
    int16_t battery_temperature_cC;
    int16_t controller_temperature_cC;
    uint16_t battery_level_percent;
    uint16_t battery_status;
    uint16_t charging_status;
    uint16_t discharging_status;
    uint16_t battery_voltage_cV;
    int32_t battery_current_cA;
} epsolar_telemetry_t;

esp_err_t epsolar_read_telemetry(epsolar_modbus_t *modbus, epsolar_telemetry_t *telemetry);