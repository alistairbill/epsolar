#!/usr/bin/env python
#
# Copyright (c) 2019, Pycom Limited.
#
# This software is licensed under the GNU GPL version 3 or any
# later version, with permitted additional terms. For more information
# see the Pycom Licence v1.0 document supplied with this file, or
# available at https://www.pycom.io/opensource/licensing
#
import struct
from machine import UART
import uasyncio as asyncio
from uModbus import functions
from micropython import const

_READ_COILS = const(0x01)
_READ_DISCRETE_INPUTS = const(0x02)
_READ_HOLDING_REGISTERS = const(0x03)
_READ_INPUT_REGISTER = const(0x04)
_WRITE_SINGLE_COIL = const(0x05)
_WRITE_SINGLE_REGISTER = const(0x06)
_WRITE_MULTIPLE_COILS = const(0x0F)
_WRITE_MULTIPLE_REGISTERS = const(0x10)

_CRC_LENGTH = const(0x02)
_ERROR_BIAS = const(0x80)
_RESPONSE_HDR_LENGTH = const(0x02)
_ERROR_RESP_LEN = const(0x05)

class Serial:
    def __init__(self, uart_id, baudrate):
        uart = UART(uart_id, baudrate=baudrate)
        self.uart = uart
        self.uart_reader = asyncio.StreamReader(uart)
        self.uart_writer = asyncio.StreamWriter(uart)

    @staticmethod
    def _crc16(data: bytearray) -> bytes:
        crc = 0xFFFF
        for char in data:
            crc ^= char
            for _ in range(0, 8):
                if (crc & 1) > 0:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return struct.pack('<H', crc)

    async def _send_receive(self, modbus_pdu: bytes, slave_addr: int, count: bool) -> bytearray:
        # flush rx buffer
        self.uart.read()

        reader = self.uart_reader
        writer = self.uart_writer

        serial_pdu = bytearray()
        serial_pdu.append(slave_addr)
        serial_pdu.extend(modbus_pdu)
        crc = self._crc16(serial_pdu)
        serial_pdu.extend(crc)

        writer.write(serial_pdu)
        await writer.drain()

        header = await asyncio.wait_for_ms(reader.readexactly(_ERROR_RESP_LEN), 100)
        if header[1] >= _ERROR_BIAS:
            raise ValueError("slave returned exception code: {:d}".format(header[2]))
        remainder = await reader.readexactly(_RESPONSE_HDR_LENGTH + 1 + header[2] + _CRC_LENGTH - _ERROR_RESP_LEN)
        response = header + remainder
        return self._validate_resp_hdr(response, slave_addr, count)

    @staticmethod
    def _validate_resp_hdr(response: bytearray, slave_addr: int, count: bool) -> bytearray:
        resp_crc = response[-_CRC_LENGTH:]
        expected_crc = Serial._crc16(response[0:len(response) - _CRC_LENGTH])
        if resp_crc[0] != expected_crc[0] or resp_crc[1] != expected_crc[1]:
            raise OSError('invalid response CRC')

        if response[0] != slave_addr:
            raise ValueError('wrong slave address')

        hdr_length = (_RESPONSE_HDR_LENGTH + 1) if count else _RESPONSE_HDR_LENGTH

        return response[hdr_length : len(response) - _CRC_LENGTH]

    # async def read_coils(self, slave_addr: int, starting_addr: int, coil_qty: int) -> bytearray:
    #     modbus_pdu = functions.read_coils(starting_addr, coil_qty)
    #     response_length = 2 + int(ceil(coil_qty / 8))
    #     return await self._send_receive(modbus_pdu, slave_addr, response_length, True)


    # async def read_discrete_inputs(self, slave_addr: int, starting_addr: int, input_qty: int) -> bytearray:
    #     modbus_pdu = functions.read_discrete_inputs(starting_addr, input_qty)
    #     response_length = 2 + int(ceil(input_qty / 8))
    #     return await self._send_receive(modbus_pdu, slave_addr, response_length, True)


    # async def read_holding_registers(self, slave_addr: int, starting_addr: int, register_qty: int) -> bytearray:
    #     modbus_pdu = functions.read_holding_registers(starting_addr, register_qty)
    #     response_length = 2 + register_qty * 2
    #     return await self._send_receive(modbus_pdu, slave_addr, response_length, True)


    async def read_input_registers(self, slave_addr: int, starting_address: int, register_quantity: int) -> bytearray:
        modbus_pdu = functions.read_input_registers(starting_address, register_quantity)
        # response_length = 2 + register_quantity * 2
        return await self._send_receive(modbus_pdu, slave_addr, True)


    # async def write_single_coil(self, slave_addr: int, output_address: int, output_value: int):
    #     modbus_pdu = functions.write_single_coil(output_address, output_value)
    #     response_length = 5
    #     resp_data = await self._send_receive(modbus_pdu, slave_addr, response_length, False)
    #     operation_status = functions.validate_resp_data(resp_data, _WRITE_SINGLE_COIL,
    #                                                     output_address, value=output_value, signed=False)
    #     return operation_status


    # async def write_single_register(self, slave_addr: int, register_address: int, register_value: int, signed=True):
    #     modbus_pdu = functions.write_single_register(register_address, register_value, signed)
    #     response_length = 5
    #     resp_data = await self._send_receive(modbus_pdu, slave_addr, response_length, False)
    #     operation_status = functions.validate_resp_data(resp_data, _WRITE_SINGLE_REGISTER,
    #                                                     register_address, value=register_value, signed=signed)
    #     return operation_status

    # async def write_multiple_coils(self, slave_addr: int, starting_address: int, output_values):
    #     modbus_pdu = functions.write_multiple_coils(starting_address, output_values)
    #     response_length = 5
    #     resp_data = await self._send_receive(modbus_pdu, slave_addr, response_length, False)
    #     operation_status = functions.validate_resp_data(resp_data, _WRITE_MULTIPLE_COILS,
    #                                                     starting_address, quantity=len(output_values))
    #     return operation_status

    # async def write_multiple_registers(self, slave_addr: int, starting_address: int, register_values, signed=True):
    #     modbus_pdu = functions.write_multiple_registers(starting_address, register_values, signed)
    #     response_length = 5
    #     resp_data = await self._send_receive(modbus_pdu, slave_addr, response_length, False)
    #     operation_status = functions.validate_resp_data(resp_data, _WRITE_MULTIPLE_REGISTERS,
    #                                                     starting_address, quantity=len(register_values))
    #     return operation_status
