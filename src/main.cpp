#include <Homie.h>
#include <ModbusMaster.h>

#define EPSOLAR_DEVICE_ID 1

uint16_t batteryPercent, batteryVoltage, batteryStatus,
  loadCurrent, loadStatus, loadVoltage,
  solarVoltage, solarCurrent;
int16_t batteryTemp, deviceTemp;
uint32_t solarPower, loadPower;
int32_t batteryCurrent;
uint8_t result;
String timeon, timeoff;

const int DEFAULT_STATS_INTERVAL = 120;
const int DEFAULT_TIMER_INTERVAL = 60 * 60;

unsigned long lastStatsSent = 0;
unsigned long lastTimerSent = 0;

ModbusMaster modbus;

HomieNode batteryCurrentNode("battery-current", "current");
HomieNode batteryLevelNode("battery-level", "fraction");
HomieNode batteryStatusNode("battery-status", "status");
HomieNode batteryTempNode("battery-temperature", "temperature");
HomieNode batteryVoltageNode("battery-voltage", "voltage");
HomieNode deviceTempNode("device-temperature", "temperature");
HomieNode loadCurrentNode("load-current", "current");
HomieNode loadPowerNode("load-power", "power");
HomieNode loadVoltageNode("load-voltage", "voltage");
HomieNode loadStatusNode("load-status", "switch");
HomieNode solarCurrentNode("solar-current", "current");
HomieNode solarPowerNode("solar-power", "power");
HomieNode solarVoltageNode("solar-voltage", "voltage");
HomieNode timerNode("timer", "time");

HomieSetting<long> statsIntervalSetting("statsInterval", "The stats interval in seconds");
HomieSetting<long> timerIntervalSetting("timerInterval", "The timer interval in seconds");

void setupHandler()
{
  batteryCurrentNode.setProperty("unit").send("A");
  batteryLevelNode.setProperty("unit").send("%");
  batteryTempNode.setProperty("unit").send("°C");
  batteryVoltageNode.setProperty("unit").send("V");

  deviceTempNode.setProperty("unit").send("°C");

  loadCurrentNode.setProperty("unit").send("A");
  loadPowerNode.setProperty("unit").send("W");
  loadVoltageNode.setProperty("unit").send("V");

  solarCurrentNode.setProperty("unit").send("A");
  solarPowerNode.setProperty("unit").send("W");
  solarVoltageNode.setProperty("unit").send("V");
}

bool timerOnHandler(const HomieRange& range, const String& value)
{
  modbus.clearTransmitBuffer();
  modbus.setTransmitBuffer(0, value.substring(6, 8).toInt());
  modbus.setTransmitBuffer(1, value.substring(3, 5).toInt());
  modbus.setTransmitBuffer(2, value.substring(0, 2).toInt());

  result = modbus.writeMultipleRegisters(0x9042, 3);
  timerNode.setProperty("on").send(value);
  return (result == modbus.ku8MBSuccess);
}

bool timerOffHandler(const HomieRange& range, const String& value)
{
  modbus.clearTransmitBuffer();
  modbus.setTransmitBuffer(0, value.substring(6, 8).toInt());
  modbus.setTransmitBuffer(1, value.substring(3, 5).toInt());
  modbus.setTransmitBuffer(2, value.substring(0, 2).toInt());

  result = modbus.writeMultipleRegisters(0x9045, 3);
  timerNode.setProperty("off").send(value);
  return (result == modbus.ku8MBSuccess);
}

bool loadStatusHandler(const HomieRange& range, const String& value)
{
  if (value != "true" && value != "false") return false;
  bool on = (value == "true");

  if (on) {
    // set manual mode
    modbus.clearTransmitBuffer();
    modbus.setTransmitBuffer(0, 0);
    result = modbus.writeMultipleRegisters(0x903D, 1);
    // set default off
    modbus.clearTransmitBuffer();
    modbus.setTransmitBuffer(0, 0);
    result = modbus.writeMultipleRegisters(0x906A, 1);
    // set load on
    result = modbus.writeSingleCoil(0x0002, 1);
    loadStatusNode.setProperty("on").send(value);

  } else {
    // set load off
    result = modbus.writeSingleCoil(0x0002, 0);

    // set automatic mode
    modbus.clearTransmitBuffer();
    modbus.setTransmitBuffer(0, 3);
    result = modbus.writeMultipleRegisters(0x903D, 1);
    loadStatusNode.setProperty("on").send(value);
  }
  return (result == modbus.ku8MBSuccess);
}

void publishTimer()
{
  result = modbus.readHoldingRegisters(0x9042, 6);
  if (result == modbus.ku8MBSuccess) {
    timeon = "";
    timeoff = "";
    if(modbus.getResponseBuffer(0x02) < 10) timeon += "0";
    timeon += modbus.getResponseBuffer(0x02);
    timeon += ":";
    if(modbus.getResponseBuffer(0x01) < 10) timeon += "0";
    timeon += modbus.getResponseBuffer(0x01);
    timeon += ":";
    if(modbus.getResponseBuffer(0x00) < 10) timeon += "0";
    timeon += modbus.getResponseBuffer(0x00);
    timerNode.setProperty("on").send(timeon);

    if(modbus.getResponseBuffer(0x05) < 10) timeoff += "0";
    timeoff += modbus.getResponseBuffer(0x05);
    timeoff += ":";
    if(modbus.getResponseBuffer(0x04) < 10) timeoff += "0";
    timeoff += modbus.getResponseBuffer(0x04);
    timeoff += ":";
    if(modbus.getResponseBuffer(0x03) < 10) timeoff += "0";
    timeoff += modbus.getResponseBuffer(0x03);
    timerNode.setProperty("off").send(timeoff);
  }
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

    batteryTemp = modbus.getResponseBuffer(0x10);
    batteryTempNode.setProperty("temperature").send(String(batteryTemp));

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

    // Discharging equipment status (0x3202)
    // bit 0: running (1) / standby (0)
    // bit 1: fault (1) / normal (0)
    loadStatus = (modbus.getResponseBuffer(0x01) & 0b1);
    loadStatusNode.setProperty("on").send((loadStatus == 1) ? "true" : "false");
  }
  modbus.clearResponseBuffer();
}

void loopHandler()
{
  if (millis() - lastStatsSent >= statsIntervalSetting.get() * 1000UL || lastStatsSent == 0) {
    publishStats();
    lastStatsSent = millis();
  }
  if (millis() - lastTimerSent >= timerIntervalSetting.get() * 1000UL || lastTimerSent == 0) {
    publishTimer();
    lastTimerSent = millis();
  }
}

void setup()
{
  timeon.reserve(9);
  timeoff.reserve(9);

  Homie.disableLogging();

  Serial.begin(115200);
  delay(10);
  modbus.begin(EPSOLAR_DEVICE_ID, Serial);

  Homie_setFirmware("epsolar", "1.0.3");
  Homie.setSetupFunction(setupHandler).setLoopFunction(loopHandler);

  batteryCurrentNode.advertise("current");
  batteryCurrentNode.advertise("unit");
  batteryLevelNode.advertise("level");
  batteryLevelNode.advertise("unit");
  batteryStatusNode.advertise("status");
  batteryTempNode.advertise("temperature");
  batteryTempNode.advertise("unit");
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

  timerNode.advertise("on").settable(timerOnHandler);
  timerNode.advertise("off").settable(timerOffHandler);

  loadStatusNode.advertise("on").settable(loadStatusHandler);

  statsIntervalSetting.setDefaultValue(DEFAULT_STATS_INTERVAL).setValidator([] (long candidate) {
    return candidate > 0;
  });

  timerIntervalSetting.setDefaultValue(DEFAULT_TIMER_INTERVAL).setValidator([] (long candidate) {
    return candidate > 0;
  });

  Homie.setup();
}

void loop()
{
  Homie.loop();
}
