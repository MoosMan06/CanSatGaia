/*
Arduino UNO wiring:

  BMP280:
    VCC <-> 3.3v
    GND <-> GND
SCK/SCL <-> 13  (required, SCK)
SDA/SDI <-> 11  (required, MOSI)
    CSB <-> 10  (~)
    SDO <-> 12  (required, MISO)

  APC220:
    VCC <-> 5v
    GND <-> GND
     RX <-> 8
     TX <-> 7
*/

#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);
//apc220 config: 450000 2400 9 9600 0

SoftwareSerial mySerial(8, 7);

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);

  Serial.println(F("BMP280 test"));
  if (!bmp.begin()) {  
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }
}

int Number;

void loop() {
  Number++;
  String PacketNumber = String(Number); //string conversion magic is currently untested

  float Temp = bmp.readTemperature(); // temperature in *C
  float Pres = bmp.readPressure(); // pressure in Pa
  
  String packet = String(PacketNumber + "/" + Temp + "/" + Pres);
  Serial.println(packet);
  mySerial.println(packet);

  delay(1000); //dies when set lower than 1s, idk why. Try increasing data transfer rate in apc220 setup
}