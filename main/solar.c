#include "solar.h"

#include <string.h>

#include "esp_log.h"

static const char *TAG = "epsolar";

static bool read_registers(
    epsolar_modbus_t *modbus,
    uint16_t address,
    uint16_t count,
    uint16_t *registers
)
{
    esp_err_t err = epsolar_modbus_read_input_registers(modbus, address, count, registers);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Modbus read 0x%04x (%u registers) failed: %s", address, count, esp_err_to_name(err));
        return false;
    }
    return true;
}

esp_err_t epsolar_read_telemetry(epsolar_modbus_t *modbus, epsolar_telemetry_t *telemetry)
{
    if (modbus == NULL || telemetry == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(telemetry, 0, sizeof(*telemetry));
    uint16_t registers[4];

    if (read_registers(modbus, 0x3100, 4, registers)) {
        telemetry->array_voltage_cV = registers[0];
        telemetry->array_current_cA = registers[1];
        telemetry->array_power_cW = ((uint32_t)registers[3] << 16) | registers[2];
        telemetry->valid |= EPSOLAR_VALID_ARRAY;
    }

    if (read_registers(modbus, 0x310c, 4, registers)) {
        telemetry->load_voltage_cV = registers[0];
        telemetry->load_current_cA = registers[1];
        telemetry->load_power_cW = ((uint32_t)registers[3] << 16) | registers[2];
        telemetry->valid |= EPSOLAR_VALID_LOAD;
    }

    if (read_registers(modbus, 0x3110, 2, registers)) {
        telemetry->battery_temperature_cC = (int16_t)registers[0];
        telemetry->controller_temperature_cC = (int16_t)registers[1];
        telemetry->valid |= EPSOLAR_VALID_TEMPERATURES;
    }

    if (read_registers(modbus, 0x311a, 1, registers)) {
        telemetry->battery_level_percent = registers[0];
        telemetry->valid |= EPSOLAR_VALID_BATTERY_LEVEL;
    }

    if (read_registers(modbus, 0x3200, 3, registers)) {
        telemetry->battery_status = registers[0];
        telemetry->charging_status = registers[1];
        telemetry->discharging_status = registers[2];
        telemetry->valid |= EPSOLAR_VALID_STATUS;
    }

    if (read_registers(modbus, 0x331a, 3, registers)) {
        telemetry->battery_voltage_cV = registers[0];
        telemetry->battery_current_cA = (int32_t)(((uint32_t)registers[2] << 16) | registers[1]);
        telemetry->valid |= EPSOLAR_VALID_BATTERY_ELECTRICAL;
    }

    return telemetry->valid == 0 ? ESP_FAIL : ESP_OK;
}