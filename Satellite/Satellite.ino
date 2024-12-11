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
*/

#include <SoftwareSerial.h>  //communication
#include <Wire.h>            //more general sensors
#include <SPI.h>             //no idea
#include <Adafruit_Sensor.h> //sensors in general
#include <Adafruit_BMP280.h> //temp/pressure sensor
#include <MPU9250_WE.h>      //gyro
#include <TinyGPS++.h>       //gps

#define MPU9250_ADDR 0x68
TinyGPSPlus gps;

MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);
//apc220 config: 450000 2400 9 9600 0

SoftwareSerial commsSerial(8, 7);
SoftwareSerial gpsSerial(4, 3);

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (gpsSerial.available())
      gps.encode(gpsSerial.read());
  } while (millis() - start < ms);
}

void setup() {
  Serial.begin(9600);
  commsSerial.begin(9600);
  gpsSerial.begin(9600);
  Wire.begin();

  if(!myMPU9250.init()){
    Serial.println("MPU9250 does not respond");
  }
  else{
    Serial.println("MPU9250 is connected");
  }
  Serial.println("Position you MPU9250 flat and don't move it - calibrating..."); //gyro calibration
  smartDelay(1000);
  myMPU9250.autoOffsets();
  Serial.println("Done!");
  myMPU9250.setAccRange(MPU9250_ACC_RANGE_2G);// can't measure more than 2G rn, calculate our max G forces later and change this to 4, 8 or 16G
  myMPU9250.enableAccDLPF(true); //digital low pass filter to reduce noise
  myMPU9250.setAccDLPF(MPU9250_DLPF_6);//also something with the low pass filter, see library example "MPU9250_angles_and_orientation" for further comments  

  Serial.println(F("BMP280 test"));
  if (!bmp.begin()) {  
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }
}

int Number;
float PosLo;
float PosLa;

void loop() {  
  Number++;
  String PacketNumber = String(Number); //string conversion magic is currently untested

  String hour = String(TinyGPSTime.hour);
  String mins = String(TinyGPSTime.minute);
  String secs = String(TinyGPSTime.second);


  String time = String(hour + ":" + mins + ":" + secs); //time
//  String pos = String(gps.location.lat() ":" gps.location.lng());

//  String gforces = String(myMPU9250.getGValues().x + ":" + myMPU9250.getGValues().y + ":" + myMPU9250.getGValues().z); // g forces
//  String angles = String(myMPU9250.getAngles().x + ":" + myMPU9250.getAngles().y + ":" + myMPU9250.getAngles().z); // angles in degrees

  float Temp = bmp.readTemperature(); // temperature in *C
  float Pres = bmp.readPressure(); // pressure in Pa

  String packet = String(PacketNumber + "/" + Temp + "/" + Pres + "/" + "gforces" + "/" + "angles" + "/" + time + "/" + "pos");
  Serial.println(packet);
  commsSerial.println(packet);

  smartDelay(1000); //dies when set lower than 1s, idk why. Try increasing data transfer rate in apc220 setup
}