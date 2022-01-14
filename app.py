import uasyncio as asyncio
import machine
import struct

import settings

from homie.device import HomieDevice, await_ready_state
from homie.node import HomieNode
from homie.property import HomieProperty
from homie.constants import FLOAT, INTEGER
from uModbus.serial import Serial

_DATA = asyncio.Event()

class Register:
    def __init__(self, address: int, quantity: int = 1, fmt: str = ">H"):
        self.address = address
        self.quantity = quantity
        self.fmt = fmt

    def parse(self, res: bytearray) -> int:
        return struct.unpack(self.fmt, res)[0]

class ShortRegister(Register):
    def __init__(self, address: int, signed: bool = False):
        super().__init__(address, 1, ">h" if signed else ">H")

class LongRegister(Register):
    def __init__(self, address: int, signed: bool = False):
        super().__init__(address, 2, ">l" if signed else ">L")
    def parse(self, res: bytearray) -> int:
        res = res[2:] + res[:2]
        return struct.unpack(self.fmt, res)[0]

class StatusRegister(Register):
    def parse(self, res: bytearray) -> int:
        return (res[1] >> 2) & 3

class Property(HomieProperty):
    def __init__(self, *args, **kwargs):
        super().__init__(restore=False, datatype=INTEGER, *args, **kwargs)

    def publish(self):
        # Only publish once
        if not _DATA.is_set():
            super().publish()

class EPSolar(HomieNode):
    def __init__(self, name="EPSolar Sensor"):
        super().__init__(id="epsolarsensor", name=name, type="epsolarsensor")
        self.modbus = Serial(0, baudrate=115200)
        self.registers = [
            (Property(id="solar-voltage", name="Solar voltage", unit="V"), ShortRegister(0x3100)),
            (Property(id="solar-current", name="Solar current", unit="A"), ShortRegister(0x3101)),
            (Property(id="solar-power", name="Solar power", unit="W"), LongRegister(0x3102)),
            (Property(id="load-voltage", name="Load voltage", unit="V"), ShortRegister(0x310c)),
            (Property(id="load-current", name="Load current", unit="A"), ShortRegister(0x310d)),
            (Property(id="load-power", name="Load power", unit="W"), LongRegister(0x310e)),
            (Property(id="air-temperature", name="Air temperature", unit="°C"), ShortRegister(0x3110, signed=True)),
            (Property(id="device-temperature", name="Device temperature", unit="°C"), ShortRegister(0x3111, signed=True)),
            (Property(id="battery-level", name="Battery level", unit="%", format="0:100"), ShortRegister(0x311a)),
            (Property(id="battery-voltage", name="Battery voltage", unit="V"), ShortRegister(0x331a)),
            (Property(id="battery-current", name="Battery current", unit="A"), LongRegister(0x331b, signed=True)),
            (Property(id="battery-status", name="Battery status"), StatusRegister(0x3201)),
        ]
        for prop, _ in self.registers:
            self.add_property(prop)
        asyncio.create_task(self.update_data())


    async def update_data(self):
        for prop, reg in self.registers:
            for _ in range(3):
                try:
                    await asyncio.sleep_ms(50) # don't send messages too fast
                    res = await asyncio.wait_for_ms(self.modbus.read_input_registers(1, reg.address, reg.quantity), 500)
                    prop.data = reg.parse(res)
                except (ValueError, asyncio.TimeoutError):
                    dprint("modbus failed, retrying")
                else:
                    break
            else:
                dprint("modbus failed", prop.id)
        _DATA.set()


    async def publish_properties(self):
        if machine.reset_cause() != machine.DEEPSLEEP_RESET:
            return await super().publish_properties()

@await_ready_state
async def wait_data():
    await _DATA.wait()

async def deep_sleep(secs: int, mqtt):
    wait_secs = 30 if machine.reset_cause() == machine.DEEPSLEEP_RESET else 60
    try:
        await asyncio.wait_for(wait_data(), wait_secs)
        dprint("deep sleep: machine ready")
        await asyncio.sleep(5)
        if mqtt.isconnected():
            await asyncio.wait_for(mqtt.disconnect(), 1)
    except asyncio.TimeoutError:
        dprint("deep sleep: timed out")
    await asyncio.sleep(1)
    rtc = machine.RTC()
    rtc.irq(trigger=rtc.ALARM0, wake=machine.DEEPSLEEP)
    rtc.alarm(rtc.ALARM0, 1000 * secs)
    machine.deepsleep()


def dprint(*args):
    if settings.DEBUG:
        print(*args)


def main():
    homie = HomieDevice(settings)
    asyncio.create_task(deep_sleep(2 * 60, homie.mqtt))
    homie.mqtt.DEBUG = settings.DEBUG
    homie.add_node(EPSolar())
    homie.run_forever()
