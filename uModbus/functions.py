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
from micropython import const

_READ_COILS = const(0x01)
_READ_DISCRETE_INPUTS = const(0x02)
_READ_HOLDING_REGISTERS = const(0x03)
_READ_INPUT_REGISTER = const(0x04)
_WRITE_SINGLE_COIL = const(0x05)
_WRITE_SINGLE_REGISTER = const(0x06)
_WRITE_MULTIPLE_COILS = const(0x0F)
_WRITE_MULTIPLE_REGISTERS = const(0x10)

# def read_coils(starting_address: int, quantity: int):
#     if not 1 <= quantity <= 2000:
#         raise ValueError('invalid number of coils')
# 
#     return struct.pack('>BHH', _READ_COILS, starting_address, quantity)
# 
# def read_discrete_inputs(starting_address: int, quantity: int):
#     if not 1 <= quantity <= 2000:
#         raise ValueError('invalid number of discrete inputs')
# 
#     return struct.pack('>BHH', _READ_DISCRETE_INPUTS, starting_address, quantity)
# 
# def read_holding_registers(starting_address: int, quantity: int):
#     if not 1 <= quantity <= 125:
#         raise ValueError('invalid number of holding registers')
# 
#     return struct.pack('>BHH', _READ_HOLDING_REGISTERS, starting_address, quantity)
# 

def read_input_registers(starting_address: int, quantity: int) -> bytes:
    if not 1 <= quantity <= 125:
        raise ValueError('invalid number of input registers')

    return struct.pack('>BHH', _READ_INPUT_REGISTER, starting_address, quantity)

# def write_single_coil(output_address: int, output_value: int):
#     if output_value not in [0x0000, 0xFF00]:
#         raise ValueError('Illegal coil value')
# 
#     return struct.pack('>BHH', _WRITE_SINGLE_COIL, output_address, output_value)
# 
# def write_single_register(register_address: int, register_value: int, signed: bool=True):
#     fmt = 'h' if signed else 'H'
# 
#     return struct.pack('>BH' + fmt, _WRITE_SINGLE_REGISTER, register_address, register_value)
# 
# def write_multiple_coils(starting_address: int, value_list):
#     sectioned_list = [value_list[i:i + 8] for i in range(0, len(value_list), 8)]
# 
#     output_value=[]
#     for byte in sectioned_list:
#         output = sum(v << i for i, v in enumerate(byte))
#         output_value.append(output)
# 
#     fmt = 'B' * len(output_value)
# 
#     return struct.pack('>BHHB' + fmt, _WRITE_MULTIPLE_COILS, starting_address,
#                         len(value_list), ((len(value_list) - 1) // 8) + 1, *output_value)
# 
# def write_multiple_registers(starting_address: int, register_values, signed: bool=True) -> bytes:
#     quantity = len(register_values)
# 
#     if not 1 <= quantity <= 123:
#         raise ValueError('invalid number of registers')
# 
#     fmt = ('h' if signed else 'H') * quantity
#     return struct.pack('>BHHB' + fmt, _WRITE_MULTIPLE_REGISTERS, starting_address,
#                         quantity, quantity * 2, *register_values)
# 
# def validate_resp_data(data: bytearray, function_code: int, address: int, value=None, quantity=None, signed: bool=True) -> bool:
#     if function_code in [_WRITE_SINGLE_COIL, _WRITE_SINGLE_REGISTER]:
#         fmt = '>H' + ('h' if signed else 'H')
#         resp_addr, resp_value = struct.unpack(fmt, data)
# 
#         if (address == resp_addr) and (value == resp_value):
#             return True
# 
#     elif function_code in [_WRITE_MULTIPLE_COILS, _WRITE_MULTIPLE_REGISTERS]:
#         resp_addr, resp_qty = struct.unpack('>HH', data)
# 
#         if (address == resp_addr) and (quantity == resp_qty):
#             return True
# 
#     return False