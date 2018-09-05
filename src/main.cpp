#include <Homie.h>
#include <ModbusMaster.h>

#define EPSOLAR_DEVICE_ID 1

uint16_t batteryPercent, batteryVoltage, batteryStatus,
  loadCurrent, loadVoltage,
  solarVoltage, solarCurrent;
int16_t airTemp, deviceTemp;
uint32_t solarPower, loadPower;
int32_t batteryCurrent;
uint8_t result;
int rtcTime[5];

const int DEFAULT_STATS_INTERVAL = 120;

unsigned long lastStatsSent = 0;

ModbusMaster modbus;

HomieNode batteryCurrentNode("battery-current", "current");
HomieNode batteryLevelNode("battery-level", "fraction");
HomieNode batteryStatusNode("battery-status", "status");
HomieNode batteryVoltageNode("battery-voltage", "voltage");

HomieNode airTempNode("air-temperature", "temperature");
HomieNode deviceTempNode("device-temperature", "temperature");

HomieNode loadCurrentNode("load-current", "current");
HomieNode loadPowerNode("load-power", "power");
HomieNode loadVoltageNode("load-voltage", "voltage");

HomieNode solarCurrentNode("solar-current", "current");
HomieNode solarPowerNode("solar-power", "power");
HomieNode solarVoltageNode("solar-voltage", "voltage");

HomieNode rtcNode("rtc", "time");

HomieSetting<long> statsIntervalSetting("statsInterval", "The stats interval in seconds");

void setupHandler()
{
  batteryCurrentNode.setProperty("unit").send("A");
  batteryLevelNode.setProperty("unit").send("%");
  batteryVoltageNode.setProperty("unit").send("V");

  airTempNode.setProperty("unit").send("°C");
  deviceTempNode.setProperty("unit").send("°C");

  loadCurrentNode.setProperty("unit").send("A");
  loadPowerNode.setProperty("unit").send("W");
  loadVoltageNode.setProperty("unit").send("V");

  solarCurrentNode.setProperty("unit").send("A");
  solarPowerNode.setProperty("unit").send("W");
  solarVoltageNode.setProperty("unit").send("V");
}

void publishStats()
{
  result = modbus.readInputRegisters(0x3100, 18);
  if (result == modbus.ku8MBSuccess) {
    solarVoltage = modbus.getResponseBuffer(0x00);
    solarVoltageNode.setProperty("voltage").send(String(solarVoltage));

    solarCurrent = modbus.getResponseBuffer(0x01);
    solarCurrentNode.setProperty("current").send(String(solarCurrent));

    solarPower = modbus.getResponseBuffer(0x02) + (modbus.getResponseBuffer(0x03) << 16);
    solarPowerNode.setProperty("power").send(String(solarPower));

    loadVoltage = modbus.getResponseBuffer(0x0C);
    loadVoltageNode.setProperty("voltage").send(String(loadVoltage));

    loadCurrent = modbus.getResponseBuffer(0x0D);
    loadCurrentNode.setProperty("current").send(String(loadCurrent));

    loadPower = modbus.getResponseBuffer(0x0E) + (modbus.getResponseBuffer(0x0F) << 16);
    loadPowerNode.setProperty("power").send(String(loadPower));

    airTemp = modbus.getResponseBuffer(0x10);
    airTempNode.setProperty("temperature").send(String(airTemp));

    deviceTemp = modbus.getResponseBuffer(0x11);
    deviceTempNode.setProperty("temperature").send(String(deviceTemp));
  }
  modbus.clearResponseBuffer();

  result = modbus.readInputRegisters(0x311A, 1);
  if (result == modbus.ku8MBSuccess) {
    batteryPercent = modbus.getResponseBuffer(0x00);
    batteryLevelNode.setProperty("level").send(String(batteryPercent));
  }
  modbus.clearResponseBuffer();

  result = modbus.readInputRegisters(0x331A, 3);
  if (result == modbus.ku8MBSuccess) {
    batteryVoltage = modbus.getResponseBuffer(0x00);
    batteryVoltageNode.setProperty("voltage").send(String(batteryVoltage));

    batteryCurrent = modbus.getResponseBuffer(0x01) + (modbus.getResponseBuffer(0x02) << 16);
    batteryCurrentNode.setProperty("current").send(String(batteryCurrent));
  }
  modbus.clearResponseBuffer();

  result = modbus.readInputRegisters(0x3201, 2);
  if (result == modbus.ku8MBSuccess) {
    // Charging equipment status (0x3201)
    // bit 0: running (1) / standby (0)
    // bit 1: fault (1) / normal (0)
    // bit 2/3: no charging (0) / float (1) / boost (2) / equalization (3)
    batteryStatus = ((modbus.getResponseBuffer(0x00) & 0b1100) >> 2);
    batteryStatusNode.setProperty("status").send(String(batteryStatus));
  }
  modbus.clearResponseBuffer();
}

bool rtcHandler(const HomieRange& range, const String& time) {
    // time: hh:mm:ss
    // rtcTime[0] seconds
    // rtcTime[1] minutes
    // rtcTime[2] hours
    // rtcTime[3] day
    // rtcTime[4] month/year
    result = modbus.readHoldingRegisters(0x9014, 2);
    if (result == modbus.ku8MBSuccess) {
        rtcTime[3] = (modbus.getResponseBuffer(0x00) >> 8);
        rtcTime[4] = modbus.getResponseBuffer(0x01);
    }
    rtcTime[2] = time.substring(0, 2).toInt();
    rtcTime[1] = time.substring(3, 5).toInt();
    rtcTime[0] = time.substring(6, 8).toInt();
    modbus.setTransmitBuffer(0, rtcTime[0] + (rtcTime[1] << 8));
    modbus.setTransmitBuffer(1, rtcTime[2] + (rtcTime[3] << 8));
    modbus.setTransmitBuffer(2, rtcTime[4]);
    result = modbus.writeMultipleRegisters(0x9013, 3);
    rtcNode.setProperty("time").send(time);
    return (result == modbus.ku8MBSuccess);
}

void loopHandler()
{
  if (millis() - lastStatsSent >= statsIntervalSetting.get() * 1000UL || lastStatsSent == 0) {
    publishStats();
    lastStatsSent = millis();
  }
}

void setup()
{
  Homie.disableLogging();

  Serial.begin(115200);
  delay(10);
  modbus.begin(EPSOLAR_DEVICE_ID, Serial);

  Homie_setFirmware("epsolar-controller", "1.0.4");
  Homie.setSetupFunction(setupHandler).setLoopFunction(loopHandler);

  batteryCurrentNode.advertise("current");
  batteryCurrentNode.advertise("unit");
  batteryLevelNode.advertise("level");
  batteryLevelNode.advertise("unit");
  batteryStatusNode.advertise("status");
  airTempNode.advertise("temperature");
  airTempNode.advertise("unit");
  batteryVoltageNode.advertise("voltage");
  batteryVoltageNode.advertise("unit");

  deviceTempNode.advertise("temperature");
  deviceTempNode.advertise("unit");

  loadCurrentNode.advertise("current");
  loadCurrentNode.advertise("unit");
  loadPowerNode.advertise("power");
  loadPowerNode.advertise("unit");
  loadVoltageNode.advertise("voltage");
  loadVoltageNode.advertise("unit");

  solarCurrentNode.advertise("unit");
  solarCurrentNode.advertise("current");
  solarPowerNode.advertise("unit");
  solarPowerNode.advertise("power");
  solarVoltageNode.advertise("voltage");
  solarVoltageNode.advertise("unit");

  rtcNode.advertise("time").settable(rtcHandler);

  statsIntervalSetting.setDefaultValue(DEFAULT_STATS_INTERVAL).setValidator([] (long candidate) {
    return candidate > 0;
  });

  Homie.setup();
}

void loop()
{
  Homie.loop();
}
