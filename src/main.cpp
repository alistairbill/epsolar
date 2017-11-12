#include "Arduino.h"
#include <ModbusMaster.h>
#include <SoftwareSerial.h>


// instantiate ModbusMaster object
ModbusMaster node;
SoftwareSerial mySerial(10, 11); // RX, TX

void setup()
{
  Serial.begin(9600);
  mySerial.begin(115200);
  node.begin(1, mySerial);
}

void loop()
{
  uint32_t result;

  result = node.readInputRegisters(0x3100, 18);
  if (result == node.ku8MBSuccess) {
    Serial.print("PV array voltage: ");
    Serial.println(node.getResponseBuffer(0x00) / 100.0f);

    Serial.print("PV array input current: ");
    Serial.println(node.getResponseBuffer(0x01) / 100.0f);

    Serial.print("Load voltage");
    Serial.println(node.getResponseBuffer(0x0C) / 100.0f);

    Serial.print("Load current");
    Serial.println(node.getResponseBuffer(0x0D) / 100.0f);

    Serial.print("Battery SOC");
    Serial.println(node.getResponseBuffer(0x1A) / 100.0f);
  } else {
    Serial.println("error");
  }
  delay(5000);
}
