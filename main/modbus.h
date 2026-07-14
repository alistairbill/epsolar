#pragma once

#include <stdint.h>

#include "esp_err.h"

typedef struct {
    void *handle;
} epsolar_modbus_t;

esp_err_t epsolar_modbus_init(epsolar_modbus_t *modbus);
void epsolar_modbus_deinit(epsolar_modbus_t *modbus);
esp_err_t epsolar_modbus_read_input_registers(
    epsolar_modbus_t *modbus,
    uint16_t start_address,
    uint16_t register_count,
    uint16_t *registers
);