#include "solar.h"

#include <string.h>

#include "esp_log.h"

static const char *TAG = "epsolar";

static bool read_registers(
    epsolar_modbus_t *modbus,
    epsolar_modbus_block_t block,
    uint16_t *registers
)
{
    esp_err_t err = epsolar_modbus_read_block(modbus, block, registers);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Modbus block %u failed: %s", block, esp_err_to_name(err));
        return false;
    }
    return true;
}

esp_err_t epsolar_read_telemetry(epsolar_modbus_t *modbus, epsolar_telemetry_t *telemetry)
{
    memset(telemetry, 0, sizeof(*telemetry));
    uint16_t registers[4];

    if (read_registers(modbus, EPSOLAR_MODBUS_BLOCK_ARRAY, registers)) {
        telemetry->array_voltage_cV = registers[0];
        telemetry->array_current_cA = registers[1];
        telemetry->array_power_cW = ((uint32_t)registers[3] << 16) | registers[2];
        telemetry->valid |= EPSOLAR_VALID_ARRAY;
    }

    if (read_registers(modbus, EPSOLAR_MODBUS_BLOCK_LOAD, registers)) {
        telemetry->load_voltage_cV = registers[0];
        telemetry->load_current_cA = registers[1];
        telemetry->load_power_cW = ((uint32_t)registers[3] << 16) | registers[2];
        telemetry->valid |= EPSOLAR_VALID_LOAD;
    }

    if (read_registers(modbus, EPSOLAR_MODBUS_BLOCK_TEMPERATURES, registers)) {
        telemetry->battery_temperature_cC = (int16_t)registers[0];
        telemetry->controller_temperature_cC = (int16_t)registers[1];
        telemetry->valid |= EPSOLAR_VALID_TEMPERATURES;
    }

    if (read_registers(modbus, EPSOLAR_MODBUS_BLOCK_BATTERY_LEVEL, registers)) {
        telemetry->battery_level_percent = registers[0];
        telemetry->valid |= EPSOLAR_VALID_BATTERY_LEVEL;
    }

    if (read_registers(modbus, EPSOLAR_MODBUS_BLOCK_STATUS, registers)) {
        telemetry->battery_status = registers[0];
        telemetry->charging_status = registers[1];
        telemetry->discharging_status = registers[2];
        telemetry->valid |= EPSOLAR_VALID_STATUS;
    }

    if (read_registers(modbus, EPSOLAR_MODBUS_BLOCK_BATTERY_ELECTRICAL, registers)) {
        telemetry->battery_voltage_cV = registers[0];
        telemetry->battery_current_cA = (int32_t)(((uint32_t)registers[2] << 16) | registers[1]);
        telemetry->valid |= EPSOLAR_VALID_BATTERY_ELECTRICAL;
    }

    return telemetry->valid == 0 ? ESP_FAIL : ESP_OK;
}