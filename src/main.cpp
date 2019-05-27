#include <Arduino.h>
#include <Homie.h>
#include <ModbusMaster.h>
#include <Timer.h>

#define FW_NAME "epsolar-controller"
#define FW_VERSION "1.1.0"

const int DEFAULT_DEEP_SLEEP_MINUTES = 2;

ModbusMaster modbus;
Timer sleepTimer;

HomieNode sensorNode("epsolarsensor", "EPSolar Sensor", "epsolarsensor");
HomieSetting<long> statsIntervalSetting("statsInterval", "The sleep duration in minutes (maximum 71 minutes)");

const int PIN_RE_NEG = D1;
const int PIN_DE = D2;

const int EPSOLAR_DEVICE_ID = 0x01;
unsigned long time_now = 0;

void prepareSleep() {
  Homie.prepareToSleep();
}

void publishStats()
{
  uint32_t solar_power, load_power;
  int32_t battery_current;
  uint16_t battery_percent, battery_voltage, battery_status, load_current,
           load_voltage, solar_voltage, solar_current;
  int16_t air_temp, device_temp;
  uint8_t result;

  result = modbus.readInputRegisters(0x3100, 18);
  if (result == modbus.ku8MBSuccess) {
    solar_voltage = modbus.getResponseBuffer(0x00);
    sensorNode.setProperty("solar-voltage").send(String(solar_voltage));

    solar_current = modbus.getResponseBuffer(0x01);
    sensorNode.setProperty("solar-current").send(String(solar_current));

    solar_power = modbus.getResponseBuffer(0x02)
                | (modbus.getResponseBuffer(0x03) << 16);
    sensorNode.setProperty("solar-power").send(String(solar_power));

    load_voltage = modbus.getResponseBuffer(0x0C);
    sensorNode.setProperty("load-voltage").send(String(load_voltage));

    load_current = modbus.getResponseBuffer(0x0D);
    sensorNode.setProperty("load-current").send(String(load_current));

    load_power = modbus.getResponseBuffer(0x0E)
               | (modbus.getResponseBuffer(0x0F) << 16);
    sensorNode.setProperty("load-power").send(String(load_power));

    air_temp = modbus.getResponseBuffer(0x10);
    sensorNode.setProperty("air-temperature").send(String(air_temp));

    device_temp = modbus.getResponseBuffer(0x11);
    sensorNode.setProperty("device-temperature").send(String(device_temp));
  }
  modbus.clearResponseBuffer();

  result = modbus.readInputRegisters(0x311A, 1);
  if (result == modbus.ku8MBSuccess) {
    battery_percent = modbus.getResponseBuffer(0x00);
    sensorNode.setProperty("battery-level").send(String(battery_percent));
  }
  modbus.clearResponseBuffer();

  result = modbus.readInputRegisters(0x331A, 3);
  if (result == modbus.ku8MBSuccess) {
    battery_voltage = modbus.getResponseBuffer(0x00);
    sensorNode.setProperty("battery-voltage").send(String(battery_voltage));

    battery_current = modbus.getResponseBuffer(0x01)
                    | (modbus.getResponseBuffer(0x02) << 16);
    sensorNode.setProperty("battery-current").send(String(battery_current));
  }
  modbus.clearResponseBuffer();

  result = modbus.readInputRegisters(0x3201, 2);
  if (result == modbus.ku8MBSuccess) {
    // Charging equipment status (0x3201)
    // bit 0: running (1) / standby (0)
    // bit 1: fault (1) / normal (0)
    // bit 2/3: no charging (0) / float (1) / boost (2) / equalization (3)
    battery_status = (modbus.getResponseBuffer(0x00) >> 2) & 0x3;
    sensorNode.setProperty("battery-status").send(String(battery_status));
  }
  modbus.clearResponseBuffer();
}


void onHomieEvent(const HomieEvent& event) {
  switch (event.type) {
    case HomieEventType::MQTT_READY:
      publishStats();
      sleepTimer.after(100, prepareSleep);
      break;
    case HomieEventType::READY_TO_SLEEP:
      Homie.doDeepSleep(statsIntervalSetting.get() * 60 * 1000 * 1000);
      break;
    default:
      break;
  }
}

void setup()
{
  Homie.disableLogging();

  Serial.begin(115200);
  modbus.begin(EPSOLAR_DEVICE_ID, Serial);

  // Set GPIO16 (= D0) pin mode to allow for deep sleep.
  // connect D0 to RST for this to work
  pinMode(D0, WAKEUP_PULLUP);

  statsIntervalSetting.setDefaultValue(DEFAULT_DEEP_SLEEP_MINUTES)
                      .setValidator([] (long candidate) {
    // 72 minutes is the maximum sleep time supported by ESP8266:
    // https://thingpulse.com/max-deep-sleep-for-esp8266/
    return candidate > 0 && candidate <= 70;
  });

  if (Homie.isConfigured()) {
    WiFi.disconnect();
  }

  Homie_setFirmware(FW_NAME, FW_VERSION);

  sensorNode.advertise("battery-current")
            .setDatatype("float")
            .setName("Battery current")
            .setUnit("A");

  sensorNode.advertise("battery-level")
            .setDatatype("float")
            .setFormat("0:100")
            .setName("Battery level")
            .setUnit("%");

  sensorNode.advertise("battery-status")
            .setDatatype("integer")
            .setName("Battery status")
            .setUnit("");

  sensorNode.advertise("battery-voltage")
            .setDatatype("float")
            .setName("Battery voltage")
            .setUnit("V");

  sensorNode.advertise("air-temperature")
            .setDatatype("float")
            .setName("Air temperature")
            .setUnit("°C");

  sensorNode.advertise("device-temperature")
            .setDatatype("float")
            .setName("Device temperature")
            .setUnit("°C");

  sensorNode.advertise("load-current")
            .setDatatype("float")
            .setName("Load current")
            .setUnit("A");

  sensorNode.advertise("load-power")
            .setDatatype("float")
            .setName("Load power")
            .setUnit("W");

  sensorNode.advertise("load-voltage")
            .setDatatype("float")
            .setName("Load voltage")
            .setUnit("V");

  sensorNode.advertise("solar-current")
            .setDatatype("float")
            .setName("Solar current")
            .setUnit("A");

  sensorNode.advertise("solar-power")
            .setDatatype("float")
            .setName("Solar power")
            .setUnit("W");

  sensorNode.advertise("solar-voltage")
            .setDatatype("float")
            .setName("Solar voltage")
            .setUnit("V");

  Homie.onEvent(onHomieEvent);
  Homie.disableLedFeedback();
  Homie.setup();
}

void loop()
{
  Homie.loop();
  sleepTimer.update();
}
