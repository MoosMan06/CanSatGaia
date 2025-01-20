// THINGS TO ADD: DEBUG_MODE setting that dissables debug messages, checksums, isValid check for packet send function, MAYBE add like a FIFO buffer for measuring at over 1 hz and sending more then 1 cycle of measurements every communication window

#include <Arduino.h>         //Arduino functions
#include <SoftwareSerial.h>  //communication
#include <Wire.h>            //more general sensors
#include <SPI.h>             //no idea but shit breaks if it's gone
#include <Adafruit_Sensor.h> //sensors in general
#include <Adafruit_BMP280.h> //temp/pressure sensor
#include <MPU6500_WE.h>      //gyro
#include <TinyGPS++.h>       //gps
#include <ML8511.h>          //UV sensor

/*
Arduino NANO wiring:        if labels on NANO are hard to read, use this diagram:
                            https://www.teachmemicro.com/arduino-nano-pinout-diagram/

  BMP280:               Temperature and pressure sensor
    VCC <-> 3.3v (required, 3.3v)
    GND <-> GND  (required, GND)
SCK/SCL <-> D13  (required, SCK)
SDI/SDA <-> D11  (required, MOSI)
    CSB <-> D10  (any PWM pin)      -line 67 to change
    SDO <-> D12  (required, MISO)

  APC220:               Communication module
    GND <-> GND  (required, GND)
    VCC <-> 5v   (required, 5v)
    RXD <-> D8   (any digital pin)  -line 76 to change
    TXD <-> D7   (any digital pin)  -line 76 to change

  MPU-92/65:            Gyroscope and accelerometer
    VCC <-> 3.3v (required, 3.3v)
    GND <-> GND  (required, GND)
    SCL <-> A5   (required, SCL)
    SDA <-> A4   (required, SDA)
    ADO <-> GND  (required, GND)
    INT <-> D2   (required, INT0)

  GY-GPSV3-NEO-M8N:     GPS module
    GND <-> GND  (required, GND)
     TX <-> D4   (any digital pin)  -line 77 to change
     RX <-> D3   (any digital pin)  -line 77 to change
    VCC <-> 5v   (required, 5v)

  ML8511:               UV sensor
    3V3 <-> 3.3v (required, 3.3v)
    GND <-> GND  (required, GND)
    OUT <-> A0   (any analog pin)   -line 61 to change
    EN  <-> 3.3v (required, 3.3v)

  GP2Y1014AU0F          Dust sensor
  [https://circuitdigest.com/sites/default/files/circuitdiagram_mic/Arduino-Dust-Sensor-Connection.png]
  [This is hard to explain through text,  PLEASE use the link above. Replace A0 with A1 and D7 with D6]

   blue <-> Resistor <-> Capacitor positive     (required, Wiring)
  green <-> GND      <-> Capacitor negative     (required, GND)
  white <--------------> D6                     (any digital pin) -line 63 to change
 yellow <--------------> GND                    (required, GND)
  black <--------------> A1                     (any analog pin)  -line 62 to change
    red <-> Resistor <-> 5v                     (required, 5v)
*/

// primitive data types
#define U_INT_8 0x00
#define U_INT_16 0x01
#define U_INT_32 0x02
#define U_INT_64 0x03
#define S_INT_8 0x04
#define S_INT_16 0x05
#define S_INT_32 0x06
#define S_INT_64 0x07
#define FLOAT 0x08
#define DOUBLE 0x09
#define BOOL 0x0A
#define CHAR 0x0B

// categorical data types
#define GPS_POS 0x01
#define G_FORCES 0x02
#define ROTATION 0x03
#define GPS_TIME 0x04
#define GPS_FIX_AGE 0x05
#define GPS_HDOP 0x06
#define GPS_NUM_OF_SATS 0x07
#define GPS_FAIL_PERCENTAGE 0x08
#define CO2_CONCENTRATION 0x09
#define TEMPERATURE 0x0A
#define PRESSURE 0x0B
#define DUST_CONCENTRATION 0x0C
#define UV_RADIATON 0x0D
#define PACKET_NUM 0x0E

// checksum types
#define NO_CHECKSUM 0x00
#define CRC8 0x10
#define CRC16 0x20
#define CRC32 0x40

// sub categories
#define NO_DIRECTION 0x00
#define DIRECTION_X 0x10
#define DIRECTION_Y 0x20
#define DIRECTION_Z 0x30

// pin definitions
#define UVPIN A0
#define DustMeasurePin A1
#define DustLedPower 6
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10
#define MPU6500_ADDR 0x68

// sensor objects
TinyGPSPlus gps;
ML8511 UVSensor(UVPIN);
MPU6500_WE gyro = MPU6500_WE(MPU6500_ADDR);
Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO, BMP_SCK);

// apc220 config: w 432600 1 9 3 0

// serial objects
SoftwareSerial commsSerial(8, 7);
SoftwareSerial GPSSerial(4, 3);

// other constants
static const int DEBUG = true;

static const int BAUD_RATE = 19200;
static const int TIMEZONE = 1;
static const int DUST_SAMPLING_TIME = 280;
static const int DUST_DELTA_TIME = 40;
static const int DUST_SLEEP_TIME = 9680;
static const byte MAGIC_NUMBER[4] = {0x47, 0x41, 0x49, 0x41};

static uint16_t packetNumber = 0;

// functions
template <typename T>
void sendPacket(T data, byte category, byte subCategory, byte dataType, byte checksum)
{

    byte packet[255]; // can't be bothered to calculate max packet size, 255 should be enough

    for (int i = 0; i < 4; i++)
    {
        packet[i] = MAGIC_NUMBER[i];
    }

    packet[4] = category | checksum;

    packet[5] = dataType | subCategory;

    uint16_t contentSize = sizeof(data);

    for (uint8_t j = 8; j <= 8 + sizeof(data); j++)
    {
        packet[j] = reinterpret_cast<byte *>(&data)[j - 8]; // idk how tf this works but it works for now
    }

    if (sizeof(data) >= 4)
    {
        if (packet[8] == MAGIC_NUMBER[0] && packet[9] == MAGIC_NUMBER[1] && packet[10] == MAGIC_NUMBER[2] && packet[11] == MAGIC_NUMBER[3])
        {
            packet[12] = 0x00;
            contentSize += 1;
        }
    }
    // I was going to add checksum logic here, but it's 05:51 and I'm tired
    packet[6] = contentSize & 0xFF;
    packet[7] = (contentSize >> 8) & 0xFF;

    for (uint16_t k = 0; k < 8 + contentSize; k++) // 8 cuz header is 8 bytes long
    {
        commsSerial.write(packet[k]);
//        Serial.print(" Y ");
    }
}

static void smartDelay(unsigned long ms)
{
    unsigned long start = millis();
    do
    {
        while (GPSSerial.available())
            gps.encode(GPSSerial.read());
    } while (millis() - start < ms);
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
    smartDelay(0);
}

static void printFloat(float value, bool isValid, int amountOfCharacters, int amountOfDecimals)
{
    if (!isValid)
    {
        while (amountOfCharacters-- > 1)
        {
            Serial.print('*');
        }
        Serial.print('*');
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
        }
    }
    smartDelay(0);
}

static void printTime(TinyGPSTime &time)
{
    if (!time.isValid())
    {
        Serial.print(F("********"));
    }
    else
    {
        char output[32];
        snprintf(output, sizeof(output), "%02d:%02d:%02d", time.hour() + TIMEZONE, time.minute(), time.second());
        Serial.print(output);
        snprintf(output, sizeof(output), "%02d_%02d_%02d", time.hour() + TIMEZONE, time.minute(), time.second());
    }
    smartDelay(0);
}

static void printXYZ(xyzFloat values, bool isValid, int amountOfCharacters, int amountOfDecimals)
{
    printFloat(values.x, isValid, amountOfCharacters, amountOfDecimals);
    Serial.print("/");
    printFloat(values.y, isValid, amountOfCharacters, amountOfDecimals);
    Serial.print("/");
    printFloat(values.z, isValid, amountOfCharacters, amountOfDecimals);

    smartDelay(0);
}

float getDustDensity()
{
    float voMeasured = 0;
    float calcVoltage = 0;
    float dustDensity = 0;

    digitalWrite(DustLedPower, LOW); // Power on the LED
    delayMicroseconds(DUST_SAMPLING_TIME);

    voMeasured = analogRead(DustMeasurePin); // Read the dust value

    delayMicroseconds(DUST_DELTA_TIME);
    digitalWrite(DustLedPower, HIGH); // Turn the LED off
    delayMicroseconds(DUST_SLEEP_TIME);

    // Calculate voltage from ADC value
    calcVoltage = voMeasured * (5.0 / 1024.0);

    // Calculate dust density using linear equation
    dustDensity = 170 * calcVoltage - 0.1;

    return dustDensity;
}

void setup()
{
    // comms
    Serial.begin(BAUD_RATE);      // begin serial communication
    commsSerial.begin(BAUD_RATE); // begin communication with APC220
    GPSSerial.begin(BAUD_RATE);   // begin communication with GPS
    Wire.begin();                 // begin communication with I2C sensors

    // gyro
    if (!gyro.init())
    {
        Serial.println("gyro is not responsive");
    }
    else
    {
        Serial.println("gyro initialized");
    }
    Serial.println("Position the gyro flat and don't move it - calibrating...");
    smartDelay(1000);
    gyro.autoOffsets();
    Serial.println("Done!");
    gyro.setAccRange(MPU6500_ACC_RANGE_8G); // set g range to 16G, options are 2G, 4G, 8G, 16G. Maybe tweak this later
    smartDelay(1000);

    // temp/pres sensor
    if (!bmp.begin())
    {
        Serial.println("temp/pres sensor is not responsive");
    }
    else
    {
        Serial.println("temp/pres sensor initialized");
    }
    smartDelay(1000);

    // dust sensor
    pinMode(DustLedPower, OUTPUT);
}

void loop()
{
//    Serial.print(F("Packet: ")); // packet number
//    printInt(packetNumber++, true, 5);
    sendPacket(packetNumber, PACKET_NUM, NO_DIRECTION, U_INT_16, NO_CHECKSUM);

//    Serial.print(F(" Temp: ")); // temperature in *C
//    printFloat(bmp.readTemperature(), bmp.begin(), 6, 2);
    sendPacket(bmp.readTemperature(), TEMPERATURE, NO_DIRECTION, FLOAT, NO_CHECKSUM);

//    Serial.print(F(" Press: ")); // pressure in Pa
//    printFloat(bmp.readPressure(), bmp.begin(), 8, 1);
    sendPacket(bmp.readPressure(), PRESSURE, NO_DIRECTION, FLOAT, NO_CHECKSUM);

//    Serial.print(F(" Angles: ")); // angle in degrees
//    printXYZ(gyro.getAngles(), gyro.init(), 7, 2);
    sendPacket(gyro.getAngles().x, ROTATION, DIRECTION_X, FLOAT, NO_CHECKSUM);
    sendPacket(gyro.getAngles().y, ROTATION, DIRECTION_Y, FLOAT, NO_CHECKSUM);
    sendPacket(gyro.getAngles().z, ROTATION, DIRECTION_Z, FLOAT, NO_CHECKSUM);

//    Serial.print(F(" Accel: ")); // acceleration in G
//    printXYZ(gyro.getGValues(), gyro.init(), 6, 2);
    sendPacket(gyro.getGValues().x, G_FORCES, DIRECTION_X, FLOAT, NO_CHECKSUM);
    sendPacket(gyro.getGValues().y, G_FORCES, DIRECTION_Y, FLOAT, NO_CHECKSUM);
    sendPacket(gyro.getGValues().z, G_FORCES, DIRECTION_Z, FLOAT, NO_CHECKSUM);

//    Serial.print(F(" UV: ")); // UV light intensity in mW/cm^2
//    printFloat(UVSensor.getUV(), UVSensor.isEnabled(), 6, 2);
    sendPacket(UVSensor.getUV(), UV_RADIATON, NO_DIRECTION, FLOAT, NO_CHECKSUM);

//    Serial.print(F(" Dust: ")); // dust density in ug/m^3
//    printFloat(getDustDensity(), true, 6, 2);
    sendPacket(getDustDensity(), DUST_CONCENTRATION, NO_DIRECTION, U_INT_16, NO_CHECKSUM);

//    Serial.print(F(" Time: ")); // time in hh:mm:ss
//    printTime(gps.time);
    sendPacket(gps.time.value(), GPS_TIME, NO_DIRECTION, U_INT_32, NO_CHECKSUM);

//    Serial.print(F(" Lat: ")); // latitude coordinate
//    printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
    sendPacket(gps.location.lat(), GPS_POS, DIRECTION_X, FLOAT, NO_CHECKSUM);

//    Serial.print(F(" Lng: ")); // longitude coordinate
//    printFloat(gps.location.lng(), gps.location.isValid(), 11, 6);
    sendPacket(gps.location.lng(), GPS_POS, DIRECTION_Y, FLOAT, NO_CHECKSUM);

//    Serial.print(F(" Alt: ")); // altitude in meters
//    printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
    sendPacket(gps.altitude.meters(), GPS_POS, DIRECTION_Z, FLOAT, NO_CHECKSUM);

//    Serial.print(F(" Age: ")); // time in ms since last valid data
//    printInt(gps.location.age(), gps.location.isValid(), 5);
    sendPacket(gps.location.age(), GPS_FIX_AGE, NO_DIRECTION, U_INT_32, NO_CHECKSUM);

//    Serial.print(F(" Location innacuracy: "));             // Horizontal Dilution Of Precision, higher number means less confidence in horizontal position. 1-2 is very accurate,
//    printFloat(gps.hdop.hdop(), gps.hdop.isValid(), 5, 2); // 2-5 is good, 5-10 is kinda alright, 10-20 is very rough and anything above 20 is useless
    sendPacket(gps.hdop.hdop(), GPS_HDOP, NO_DIRECTION, FLOAT, NO_CHECKSUM);

//    Serial.print(F(" Sats: ")); // amount of satellites we are receiving data from
//    printInt(gps.satellites.value(), gps.satellites.isValid(), 2);
    sendPacket((uint8_t)gps.satellites.value(), GPS_NUM_OF_SATS, NO_DIRECTION, U_INT_8, NO_CHECKSUM);

    unsigned long passed = gps.passedChecksum();
    unsigned long failed = gps.failedChecksum();
    unsigned long total = passed + failed;
    float failPercentage = (failed / (float)total) * 100.0;
    sendPacket(failPercentage, GPS_FAIL_PERCENTAGE, NO_DIRECTION, FLOAT, NO_CHECKSUM);

//    Serial.print(F(" GPS Fails %: "));
//    printFloat(failPercentage, true, 5, 1);

//    Serial.println();

    if (millis() > 5000 && gps.charsProcessed() < 10)
    {
//        Serial.println(F("No GPS data received: check wiring"));
    }
    else
    {
        smartDelay(1000);
    }
}
