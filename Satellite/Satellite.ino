#include <SoftwareSerial.h>  //communication
#include <Wire.h>            //more general sensors
#include <SPI.h>             //no idea
#include <Adafruit_Sensor.h> //sensors in general
#include <Adafruit_BMP280.h> //temp/pressure sensor
#include <MPU9250_WE.h>      //gyro
#include <TinyGPS++.h>       //gps

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

  MPU-92/65
    VCC <-> 3.3v
    GND <-> GND
    SCL <-> A5  (required, hardcoded in library)
    ADS <-> A4  (required, hardcoded in library)
    ADO <-> GND (required, idk why)
    INI <-> 2   (required, hardcoded in library)

  GY-GPSV3-NEO-M8N:
    VCC <-> 5v
    GND <-> GND
     RX <-> 3
     TX <-> 4
*/

#define MPU9250_ADDRESS 0x68
TinyGPSPlus gps;
MPU9250_WE gyro = MPU9250_WE(MPU9250_ADDRESS);

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO, BMP_SCK);
// apc220 config: 450000 2400 9 9600 0

SoftwareSerial commsSerial(8, 7);
SoftwareSerial GPSSerial(4, 3);

static void printInt(unsigned long value, bool isValid, int amountOfCharacters);
static void printFloat(float value, bool isValid, int amountOfCharacters, int amountOfDecimals);
static void printTime(TinyGPSTime &t);
static void smartDelay(unsigned long ms);
static void printXYZ(xyzFloat values, int amountOfCharacters, int amountOfDecimals);

static const int BAUD_RATE = 9600;
static const int TIMEZONE = 1;

void setup()
{
  Serial.begin(BAUD_RATE);      // begin serial communication
  commsSerial.begin(BAUD_RATE); // begin communication with APC220
  GPSSerial.begin(BAUD_RATE);   // begin communication with GPS
  Wire.begin();            // begin communication with I2C sensors

  Serial.println("Position you MPU9250 flat and don't move it - calibrating..."); // gyro calibration
  smartDelay(1000);
  gyro.autoOffsets();
  Serial.println("Done!");
  gyro.setAccRange(MPU9250_ACC_RANGE_8G); // set g range to 16G, options are 2G, 4G, 8G, 16G. Maybe tweak this later
  gyro.enableAccDLPF(true);               // digital low pass filter to reduce noise
  gyro.setAccDLPF(MPU9250_DLPF_6);        // also something with the low pass filter, see library example "MPU9250_angles_and_orientation" for further comments
  bmp.begin();
  gyro.init();
}
unsigned int packetNumber;
float temp, pres;
void loop()
{
  Serial.print(F("Packet: ")); // packet number
  printInt(packetNumber++, true, 5);

  Serial.print(F(" Temp: ")); // temperature in *C
  printFloat(temp, bmp.begin(), 6, 2);

  Serial.print(F(" Press: ")); // pressure in Pa
  printFloat(pres, bmp.begin(), 8, 1);

  Serial.print(F(" Angles: ")); // angle in degrees
  printXYZ(gyro.getAngles(), 7, 2);

  Serial.print(F(" Accel: ")); // acceleration in G
  printXYZ(gyro.getGValues(), 6, 2);

  Serial.print(F(" Time: ")); // time in hh:mm:ss
  printTime(gps.time);

  Serial.print(F(" Lat: ")); // latitude coordinate
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);

  Serial.print(F(" Lng: ")); // longitude coordinate
  printFloat(gps.location.lng(), gps.location.isValid(), 11, 6);

  Serial.print(F(" Alt: ")); // altitude in meters
  printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);

  Serial.print(F(" Age: ")); // time in ms since last valid data
  printInt(gps.location.age(), gps.location.isValid(), 5);

  Serial.print(F(" Error: ")); // Horizontal Dilution Of Precision, higher number means less confidence in horizontal position. 1-2 is very accurate, 
  printFloat(gps.hdop.hdop(), gps.hdop.isValid(), 5, 2); // 2-5 is good, 5-10 is kinda alright, 10-20 is very rough and anything above 20 is useless

  Serial.print(F(" Sats: ")); // amount of satellites we are receiving data from
  printInt(gps.satellites.value(), gps.satellites.isValid(), 2);

  unsigned long passed = gps.passedChecksum();
  unsigned long failed = gps.failedChecksum();
  unsigned long total = passed + failed;
  float successPercentage = (passed / (float)total) * 100.0;

  Serial.print(F(" Fail%: "));
  printFloat(successPercentage, true, 5, 1);

  commsSerial.println();
  Serial.println();

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS data received: check wiring"));
  }
  else
  {
    smartDelay(1000);
  }
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    temp = bmp.readTemperature();
    pres = bmp.readPressure();
    while (GPSSerial.available())
      gps.encode(GPSSerial.read());
  } while (millis() - start < ms);
}

static void printFloat(float value, bool isValid, int amountOfCharacters, int amountOfDecimals)
{
  if (!isValid)
  {
    while (amountOfCharacters-- > 1)
    {
      Serial.print('*');
      commsSerial.print('*');
    }
    Serial.print('*');
    commsSerial.print('*');
  }
  else
  {
    Serial.print(value, amountOfDecimals);
    commsSerial.print(value, amountOfDecimals);
    int valueWithoutDecimals = abs((int)value);
    int valueLength = amountOfDecimals + (value < 0.0 ? 2 : 1); // . and -
    do
    {
      valueWithoutDecimals /= 10;
      valueLength++;
    } while (valueWithoutDecimals > 0);
    for (int i = valueLength; i < amountOfCharacters; ++i)
    {
      Serial.print('_');
      commsSerial.print('_');
    }
  }
  smartDelay(0);
}

static void printInt(unsigned long value, bool isValid, int amountOfCharacters)
{
  char output[32] = "*****************";
  if (isValid)
    snprintf(output, sizeof(output), "%lu", value);
  output[amountOfCharacters] = 0;
  for (int i = strlen(output); i < amountOfCharacters; ++i)
    output[i] = '_';
  Serial.print(output);
  commsSerial.print(output);
  smartDelay(0);
}

static void printTime(TinyGPSTime &time)
{
  if (!time.isValid())
  {
    Serial.print(F("********"));
    commsSerial.print(F("********"));
  }
  else
  {
    char output[32];
    snprintf(output, sizeof(output), "%02d:%02d:%02d", time.hour() + TIMEZONE, time.minute(), time.second());
    Serial.print(output);
    snprintf(output, sizeof(output), "%02d_%02d_%02d", time.hour() + TIMEZONE, time.minute(), time.second());
    commsSerial.print(output);
  }
  smartDelay(0);
}

static void printXYZ(xyzFloat values, int amountOfCharacters, int amountOfDecimals)
{
  printFloat(values.x, true, amountOfCharacters, amountOfDecimals);
  Serial.print("/");
  commsSerial.print("_");
  printFloat(values.y, true, amountOfCharacters, amountOfDecimals);
  Serial.print("/");
  commsSerial.print("_");
  printFloat(values.z, true, amountOfCharacters, amountOfDecimals);

  smartDelay(0);
}
