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

_CRC_LENGTH = const(2)
_ERROR_BIAS = const(0x80)
_ERROR_RESP_LEN = const(5)


class IncompleteReadError(EOFError):
    def __init__(self, partial, expected):
        super().__init__(f'{len(partial)} bytes read on a total of {expected} expected bytes')
        self.partial = partial
        self.expected = expected


class Serial:
    def __init__(self, uart_id: int, baudrate: int):
        uart = UART(uart_id, baudrate=baudrate)
        self.uart = uart
        self.uart_reader = asyncio.StreamReader(uart)
        self.uart_writer = asyncio.StreamWriter(uart)


    @staticmethod
    @micropython.viper
    def _crc16(data) -> int:
        crc = 0xFFFF
        buf = ptr8(data)
        n = int(len(data))
        for i in range(n):
            crc ^= buf[i]
            for _ in range(0, 8):
                if (crc & 1) > 0:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc


    async def _read_exactly(self, n: int):
        buffer = bytearray()
        i = n
        while i:
            res = await self.uart_reader.read(i)
            if res is not None:
                if not res:
                    raise IncompleteReadError(buffer, n)
                buffer += res
                i -= len(res)
        return buffer


    async def _send_receive(self, modbus_pdu: bytes, slave_addr: int, length) -> bytearray:
        # flush rx buffer
        self.uart.read()

        writer = self.uart_writer

        serial_pdu = bytearray()
        serial_pdu.append(slave_addr)
        serial_pdu.extend(modbus_pdu)
        crc = struct.pack("<H", self._crc16(serial_pdu))
        serial_pdu.extend(crc)

        writer.write(serial_pdu)
        await writer.drain()

        header_length, body_length = length
        try:
            response = await self._read_exactly(header_length + body_length + _CRC_LENGTH)
            return self._validate_resp_hdr(response, slave_addr, length)
        except IncompleteReadError as err:
            if err.partial[1] >= _ERROR_BIAS:
                raise ValueError(f"slave returned exception code: {err.partial[2]}") from err
            return self._validate_resp_hdr(err.partial, slave_addr, length)


    @staticmethod
    def _validate_resp_hdr(response: bytearray, slave_addr: int, length) -> bytearray:
        resp_crc = struct.unpack("<H", response[-_CRC_LENGTH:])[0]
        expected_crc = Serial._crc16(response[:-_CRC_LENGTH])
        if resp_crc != expected_crc:
            raise ValueError(f"invalid response CRC: expected {expected_crc} got {resp_crc}")

        if response[0] != slave_addr:
            raise ValueError(f"wrong slave address expected {slave_addr} got {response[0]}")

        if response[1] >= _ERROR_BIAS:
            raise ValueError(f"slave returned exception code: {response[2]}")

        header_length, body_length = length
        return response[header_length : header_length + body_length]

    # async def read_coils(self, slave_addr: int, starting_addr: int, coil_qty: int) -> bytearray:
    #     modbus_pdu = functions.read_coils(starting_addr, coil_qty)
    #     response_length = 2 + int(ceil(coil_qty / 8))
    #     length = (3, int(ceil(coil_qty / 8))
    #     return await self._send_receive(modbus_pdu, slave_addr, response_length, True)


    # async def read_discrete_inputs(self, slave_addr: int, starting_addr: int, input_qty: int) -> bytearray:
    #     modbus_pdu = functions.read_discrete_inputs(starting_addr, input_qty)
    #     length = (3, int(ceil(input_qty / 8))
    #     return await self._send_receive(modbus_pdu, slave_addr, response_length, True)


    # async def read_holding_registers(self, slave_addr: int, starting_addr: int, register_qty: int) -> bytearray:
    #     modbus_pdu = functions.read_holding_registers(starting_addr, register_qty)
    #     length = (3, register_qty * 2)
    #     return await self._send_receive(modbus_pdu, slave_addr, response_length, True)


    async def read_input_registers(self, slave_addr: int, starting_address: int, register_quantity: int) -> bytearray:
        modbus_pdu = functions.read_input_registers(starting_address, register_quantity)
        length = (3, register_quantity * 2)
        return await self._send_receive(modbus_pdu, slave_addr, length)


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
