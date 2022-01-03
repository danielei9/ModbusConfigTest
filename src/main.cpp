#include <Arduino.h>
#define TTGO_SOFTWARE_SERIAL 0
#include "Modbus.hpp"
#include <SoftwareSerial.h>
const int modbusBaudRate = 19200;
const int SSRxPin = 35; //(Rx on RS485 adapter)DI
const int SSTxPin = 25; //(Tx on RS485 adapter) RO
SoftwareSerial modbusSerial(SSRxPin, SSTxPin);

void initModbus();
uint8_t* getModbusData();
uint32_t knownCRC32 = 0xD9971BA9;
// Construct the modbus instance
Modbus modbus;
void initModbus();
#define DEBUG 0
unsigned int mill=0;
bool timerTrue(unsigned long lastmillis_, int interval);

void setup()
{
  Serial.begin(115200);
  initModbus();
  Serial.print("Setting BaudrateModbus to: ");
  Serial.println(modbusBaudRate);
}
void loop()
{
  if (timerTrue(mill, 31000)){ 
    // envia los datos del modbus (Sensores conectados al modbus)
    byte *arrayData = new byte[52];
    //byte Modbus::command[8] = {0x01, 0x04, 0x00, 0x00, 0x00, 0x03, 0xB0, 0x0B};
    byte temp = modbus.sendCommand(modbus.command,sizeof(modbus.command));
    mill = millis();
  }
}

// (lastMillis:R , TX_interval:R) -> timerTrue -> bool
bool timerTrue(unsigned long lastmillis_, int interval)
{
    if (millis() > (lastmillis_ + interval))
        return true;

    return false;
}
void initModbus()
{
  modbusSerial.begin(modbusBaudRate);
  modbus.begin(0x01, modbusSerial, DEREPin);
}
