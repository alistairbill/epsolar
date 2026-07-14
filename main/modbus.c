#include "modbus.h"

#include <stddef.h>

#include "driver/uart.h"
#include "esp_check.h"
#include "esp_modbus_master.h"

#define MODBUS_READ_INPUT_REGISTERS 0x04
#define MODBUS_RESPONSE_TIMEOUT_MS 500

static const char *TAG = "epsolar_modbus";

esp_err_t epsolar_modbus_init(epsolar_modbus_t *modbus)
{
    ESP_RETURN_ON_FALSE(modbus != NULL, ESP_ERR_INVALID_ARG, TAG, "Missing Modbus context");
    ESP_RETURN_ON_FALSE(modbus->handle == NULL, ESP_ERR_INVALID_STATE, TAG, "Modbus already initialized");

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

    ESP_RETURN_ON_ERROR(mbc_master_create_serial(&communication, &modbus->handle), TAG, "Creating Modbus master");

    esp_err_t err = uart_set_pin(
        CONFIG_EPSOLAR_MODBUS_UART_PORT,
        CONFIG_EPSOLAR_MODBUS_TX_GPIO,
        CONFIG_EPSOLAR_MODBUS_RX_GPIO,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    );
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

esp_err_t epsolar_modbus_read_input_registers(
    epsolar_modbus_t *modbus,
    uint16_t start_address,
    uint16_t register_count,
    uint16_t *registers
)
{
    ESP_RETURN_ON_FALSE(modbus != NULL && modbus->handle != NULL, ESP_ERR_INVALID_STATE, TAG, "Modbus not initialized");
    ESP_RETURN_ON_FALSE(registers != NULL && register_count > 0, ESP_ERR_INVALID_ARG, TAG, "Invalid read buffer");

    mb_param_request_t request = {
        .slave_addr = CONFIG_EPSOLAR_MODBUS_SLAVE_ADDRESS,
        .command = MODBUS_READ_INPUT_REGISTERS,
        .reg_start = start_address,
        .reg_size = register_count,
    };
    return mbc_master_send_request(modbus->handle, &request, registers);
}
