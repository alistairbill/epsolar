#include <ArduinoOTA.h>
#include <ESP8266mDNS.h>
#include <ESP8266WiFi.h>
#include <ModbusMaster.h>
#include <NTPClient.h>
#include <PubSubClient.h>
#include <WiFiUdp.h>

#define EPSOLAR_DEVICE_ID 1

const char* ssid = "no";
const char* password = "no";
const char* mqtt_server = "rpi3.localdomain";
const long interval = 120000;
const long timeUpdateInterval = 3600000;

unsigned int solarVoltage, solarCurrent, batteryTemp, deviceTemp, loadVoltage,
    loadCurrent, batteryPercent, batteryVoltage, status;
unsigned long solarPower, loadPower, batteryCurrent;
unsigned long previousMillis = 0;
uint8_t result;
char buf[10];

WiFiClient espClient;
PubSubClient client(espClient);
ModbusMaster node;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

void reconnect()
{
  while (!client.connected()) {
    if(client.connect("ESP8266Client")) {
      client.subscribe("pond/load/status");
    } else {
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length)
{
  if (strcmp(topic, "pond/load/status") == 0) {
    if ((char)payload[0] == '1' && status == 0) {
      result = node.writeSingleCoil(0x0001, true);
      result = node.writeSingleCoil(0x0002, true);
      status = 1;
    } else if ((char)payload[0] == '0' && status == 1) {
      result = node.writeSingleCoil(0x0002, false);
      result = node.writeSingleCoil(0x0001, false);
      status = 0;
    }
  }
}

void setup()
{
  Serial.begin(115200);
  delay(10);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  node.begin(EPSOLAR_DEVICE_ID, Serial);
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  timeClient.begin();
  ArduinoOTA.begin();
}

void loop()
{
  ArduinoOTA.handle();

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    if (currentMillis - previousMillis >= timeUpdateInterval) {
      unsigned int day = 1;
      result = node.readInputRegisters(0x9014, 1);
      if (result == node.ku8MBSuccess) {
        day = node.getResponseBuffer(0x00);
      }
      timeClient.update();
      node.setTransmitBuffer(0, timeClient.getSeconds() + (timeClient.getMinutes() << 8));
      node.setTransmitBuffer(0, timeClient.getHours() + (day << 8));
      result = node.writeMultipleRegisters(0x9013, 2);
    }

    previousMillis = currentMillis;

    result = node.readInputRegisters(0x3100, 18);
    if (result == node.ku8MBSuccess) {
      solarVoltage = node.getResponseBuffer(0x00);
      itoa(solarVoltage, buf, 10);
      client.publish("pond/solar/voltage", buf);

      solarCurrent = node.getResponseBuffer(0x01);
      itoa(solarCurrent, buf, 10);
      client.publish("pond/solar/current", buf);

      solarPower = node.getResponseBuffer(0x02) + (node.getResponseBuffer(0x03) << 16);
      itoa(solarPower, buf, 10);
      client.publish("pond/solar/power", buf);

      loadVoltage = node.getResponseBuffer(0x0C);
      itoa(loadVoltage, buf, 10);
      client.publish("pond/load/voltage", buf);

      loadCurrent = node.getResponseBuffer(0x0D);
      itoa(loadCurrent, buf, 10);
      client.publish("pond/load/current", buf);

      loadPower = node.getResponseBuffer(0x0E) + (node.getResponseBuffer(0x0F) << 16);
      itoa(loadPower, buf, 10);
      client.publish("pond/load/power", buf);

      batteryTemp = node.getResponseBuffer(0x10);
      itoa(batteryTemp, buf, 10);
      client.publish("pond/battery/temp", buf);

      deviceTemp = node.getResponseBuffer(0x11);
      itoa(deviceTemp, buf, 10);
      client.publish("pond/temp", buf);
    }
    node.clearResponseBuffer();
    delay(5);

    result = node.readInputRegisters(0x311A, 1);
    if (result == node.ku8MBSuccess) {
      batteryPercent = node.getResponseBuffer(0x00);
      itoa(batteryPercent, buf, 10);
      client.publish("pond/battery/percent", buf);
    }
    node.clearResponseBuffer();
    delay(5);

    result = node.readInputRegisters(0x331A, 3);
    if (result == node.ku8MBSuccess) {
      batteryVoltage = node.getResponseBuffer(0x00);
      itoa(batteryVoltage, buf, 10);
      client.publish("pond/battery/voltage", buf);

      batteryCurrent = node.getResponseBuffer(0x01) + (node.getResponseBuffer(0x02) << 16);
      itoa(batteryCurrent, buf, 10);
      client.publish("pond/battery/current", buf);
    }
    node.clearResponseBuffer();

    result = node.readInputRegisters(0x3202, 1);
    if (result == node.ku8MBSuccess) {
      status = node.getResponseBuffer(0x00);
      itoa(status, buf, 10);
      client.publish("pond/load/status", buf);
    }
    node.clearResponseBuffer();
  }
}
