import uasyncio as asyncio
import machine
import struct

import settings

from homie.device import HomieDevice, _MESSAGE
from homie.node import HomieNode
from homie.property import HomieProperty
from homie.constants import FLOAT, INTEGER
from uModbus.serial import Serial


class EPSolar(HomieNode):
    def __init__(self, name="EPSolar Sensor"):
        super().__init__(id="epsolarsensor", name=name, type="epsolarsensor")
        self.modbus = Serial(0, 115200)
        uint16 = (1, ">H")
        uint32 = (2, ">L")
        int16 = (1, ">h")
        int32 = (2, ">l")
        status = (1, ">H")
        self.registers = [
            (HomieProperty(restore=False, id="solar-voltage", datatype=FLOAT, name="Solar voltage", unit="V"), 0x3100, uint16),
            (HomieProperty(restore=False, id="solar-current", datatype=FLOAT, name="Solar current", unit="A"), 0x3101, uint16),
            (HomieProperty(restore=False, id="solar-power", datatype=FLOAT, name="Solar power", unit="W"), 0x3102, uint32),
            (HomieProperty(restore=False, id="load-voltage", datatype=FLOAT, name="Load voltage", unit="V"), 0x310c, uint16),
            (HomieProperty(restore=False, id="load-current", datatype=FLOAT, name="Load current", unit="A"), 0x310d, uint16),
            (HomieProperty(restore=False, id="load-power", datatype=FLOAT, name="Load power", unit="W"), 0x310e, uint32),
            (HomieProperty(restore=False, id="air-temperature", datatype=FLOAT, name="Air temperature", unit="°C"), 0x3110, int16),
            (HomieProperty(restore=False, id="device-temperature", datatype=FLOAT, name="Device temperature", unit="°C"), 0x3111, int16),
            (HomieProperty(restore=False, id="battery-level", datatype=FLOAT, name="Battery level", unit="%", format="0:100"), 0x311a, uint16),
            (HomieProperty(restore=False, id="battery-voltage", datatype=FLOAT, name="Battery voltage", unit="V"), 0x331a, uint16),
            (HomieProperty(restore=False, id="battery-current", datatype=FLOAT, name="Battery current", unit="A"), 0x331b, int32),
            (HomieProperty(restore=False, id="battery-status", datatype=INTEGER, name="Battery status"), 0x3201, status),
        ]
        for prop, _, _ in self.registers:
            self.add_property(prop)
        asyncio.create_task(self.update_data())


    async def update_data(self):
        for prop, address, (quantity, fmt) in self.registers:
            try:
                res = await self.modbus.read_input_registers(1, address, quantity)
                prop.data = struct.unpack(fmt, res)[0]
            except asyncio.TimeoutError:
                dprint("modbus: timed out")


    async def publish_properties(self):
        if machine.reset_cause() != machine.DEEPSLEEP_RESET:
            return await super().publish_properties()

async def deep_sleep(secs: int, mqtt):
    try:
        await asyncio.wait_for(_MESSAGE.wait(), 20)
        dprint("deep sleep: machine ready")

        await asyncio.wait_for(mqtt.disconnect(), 1)
    except asyncio.TimeoutError:
        dprint("deep sleep: timed out")
    await asyncio.sleep_ms(500)
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
