#pragma once

#include <stdint.h>

#include "esp_err.h"
#ifdef CONFIG_PM_ENABLE
#include "esp_pm.h"
#endif

typedef enum {
    EPSOLAR_MODBUS_BLOCK_ARRAY,
    EPSOLAR_MODBUS_BLOCK_LOAD,
    EPSOLAR_MODBUS_BLOCK_TEMPERATURES,
    EPSOLAR_MODBUS_BLOCK_BATTERY_LEVEL,
    EPSOLAR_MODBUS_BLOCK_STATUS,
    EPSOLAR_MODBUS_BLOCK_BATTERY_ELECTRICAL,
    EPSOLAR_MODBUS_BLOCK_COUNT,
} epsolar_modbus_block_t;

typedef struct {
    void *handle;
#ifdef CONFIG_PM_ENABLE
    /* Blocks automatic light sleep for the duration of a transaction: UART RX
     * bytes are lost while the chip sleeps and esp-modbus takes no PM lock. */
    esp_pm_lock_handle_t pm_lock;
#endif
} epsolar_modbus_t;

esp_err_t epsolar_modbus_init(epsolar_modbus_t *modbus);
esp_err_t epsolar_modbus_read_block(
    epsolar_modbus_t *modbus,
    epsolar_modbus_block_t block,
    uint16_t *registers
);