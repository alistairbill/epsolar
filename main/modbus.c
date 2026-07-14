#include "modbus.h"

#include <stddef.h>

#include "driver/uart.h"
#include "esp_modbus_master.h"

#define MODBUS_RESPONSE_TIMEOUT_MS 500

#define BLOCK_DESCRIPTOR(block_id, key, address, count)       \
    {                                                          \
        .cid = (block_id),                                     \
        .param_key = (key),                                    \
        .param_units = "",                                     \
        .mb_slave_addr = CONFIG_EPSOLAR_MODBUS_SLAVE_ADDRESS,  \
        .mb_param_type = MB_PARAM_INPUT,                       \
        .mb_reg_start = (address),                             \
        .mb_size = (count),                                    \
        .param_type = PARAM_TYPE_BIN,                          \
        .param_size = (count) * sizeof(uint16_t),              \
        .access = PAR_PERMS_READ,                              \
    }

static const mb_parameter_descriptor_t register_blocks[] = {
    BLOCK_DESCRIPTOR(EPSOLAR_MODBUS_BLOCK_ARRAY, "array", 0x3100, 4),
    BLOCK_DESCRIPTOR(EPSOLAR_MODBUS_BLOCK_LOAD, "load", 0x310c, 4),
    BLOCK_DESCRIPTOR(EPSOLAR_MODBUS_BLOCK_TEMPERATURES, "temperatures", 0x3110, 2),
    BLOCK_DESCRIPTOR(EPSOLAR_MODBUS_BLOCK_BATTERY_LEVEL, "battery_level", 0x311a, 1),
    BLOCK_DESCRIPTOR(EPSOLAR_MODBUS_BLOCK_STATUS, "status", 0x3200, 3),
    BLOCK_DESCRIPTOR(EPSOLAR_MODBUS_BLOCK_BATTERY_ELECTRICAL, "battery_electrical", 0x331a, 3),
};

esp_err_t epsolar_modbus_init(epsolar_modbus_t *modbus)
{
    if (modbus == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (modbus->handle != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    mb_communication_info_t communication = {
        .ser_opts = {
            .mode = MB_RTU,
            .port = CONFIG_EPSOLAR_MODBUS_UART_PORT,
            .uid = 0,
            .response_tout_ms = MODBUS_RESPONSE_TIMEOUT_MS,
            .baudrate = CONFIG_EPSOLAR_MODBUS_BAUD_RATE,
            .data_bits = UART_DATA_8_BITS,
            .stop_bits = UART_STOP_BITS_1,
            .parity = UART_PARITY_DISABLE,
        },
    };

    esp_err_t err = mbc_master_create_serial(&communication, &modbus->handle);
    if (err != ESP_OK) {
        return err;
    }

    err = uart_set_pin(
        CONFIG_EPSOLAR_MODBUS_UART_PORT,
        CONFIG_EPSOLAR_MODBUS_TX_GPIO,
        CONFIG_EPSOLAR_MODBUS_RX_GPIO,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    );
    if (err == ESP_OK) {
        err = mbc_master_set_descriptor(
            modbus->handle,
            register_blocks,
            EPSOLAR_MODBUS_BLOCK_COUNT
        );
    }
    if (err == ESP_OK) {
        err = mbc_master_start(modbus->handle);
    }
    if (err != ESP_OK) {
        mbc_master_delete(modbus->handle);
        modbus->handle = NULL;
    }
    return err;
}

void epsolar_modbus_deinit(epsolar_modbus_t *modbus)
{
    if (modbus == NULL || modbus->handle == NULL) {
        return;
    }
    mbc_master_delete(modbus->handle);
    modbus->handle = NULL;
}

esp_err_t epsolar_modbus_read_block(
    epsolar_modbus_t *modbus,
    epsolar_modbus_block_t block,
    uint16_t *registers
)
{
    if (modbus == NULL || modbus->handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (registers == NULL || (unsigned)block >= EPSOLAR_MODBUS_BLOCK_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t parameter_type = 0;
    esp_err_t err = mbc_master_get_parameter(
        modbus->handle,
        block,
        (uint8_t *)registers,
        &parameter_type
    );
    if (err == ESP_OK && parameter_type != PARAM_TYPE_BIN) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    return err;
}
