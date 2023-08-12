#ifndef __MODBUS__H__
#define __MODBUS__H__
#include <esp_err.h>

esp_err_t modbus_init(void);
int modbus_read_input_registers(uint8_t slave_id, uint16_t start_address, uint16_t num_registers, uint16_t *out_buf);

#endif