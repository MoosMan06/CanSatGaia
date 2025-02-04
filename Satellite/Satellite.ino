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

// data types
#define GPS_POS 0x01
#define G_FORCES 0x02
#define ROTATION 0x03
#define TIME 0x04
#define GPS_FIX_AGE 0x05
#define GPS_HDOP 0x06
#define GPS_NUM_OF_SATS 0x07
#define GPS_FAIL_PERCENTAGE 0x08
#define CO2_CONCENTRATION 0x09
#define TEMPERATURE 0x0A
#define PRESSURE 0x0B
#define DUST_CONCENTRATION 0x0C
#define UV_RADIATON 0x0D

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

// apc220 config: w 432600 1 9 0 0

// serial objects
SoftwareSerial commsSerial(8, 7);
SoftwareSerial GPSSerial(4, 3);

// other constants
static const int DEBUG = true; // not implemented yet

static const int BAUD_RATE = 9600;
static const int TIMEZONE = 1;
static const int DUST_SAMPLING_TIME = 280;
static const int DUST_DELTA_TIME = 40;
static const int DUST_SLEEP_TIME = 9680;
static const byte MAGIC_NUMBER[4] = {0x67, 0x61, 0x69, 0x61};

static uint16_t packetNumber = 0;

// functions
void escapeMagicNumbers(byte *array, size_t &arraySize, const size_t maxSize, const byte *pattern, size_t patternSize)
{
    size_t i = 0;

    while (i <= arraySize - patternSize)
    {
        // Check for a match at the current position
        bool match = true;
        for (size_t j = 0; j < patternSize; j++)
        {
            if (array[i + j] != pattern[j])
            {
                match = false;
                break;
            }
        }

        // If a match is found
        if (match)
        {
            // Ensure there's room in the array to add a byte
            if (arraySize + 1 > maxSize)
            {
                Serial.println("Not enough space to add 0x00 byte.");
                return;
            }

            // Shift elements to the right to make space for 0x00
            for (size_t j = arraySize; j > i + patternSize; j--)
            {
                array[j] = array[j - 1];
            }

            // Insert 0x00 after the pattern
            array[i + patternSize] = 0x00;
            arraySize++;

            // Move past the newly inserted 0x00
            i += patternSize + 1; // Skip past the inserted 0x00
        }
        else
        {
            i++;
        }
    }
}

template <typename T>
void sendPacket(const T *data, size_t dataCount, byte category, uint16_t packetNumber)
{
    // Calculate raw data size
    size_t dataSize = sizeof(T) * dataCount;

    // Ensure the maximum size fits within the buffer
    const size_t maxDataSize = 16;
    byte dataArray[maxDataSize];
    if (dataSize > maxDataSize)
    {
        Serial.println("Data too large to fit in the buffer.");
        return;
    }

    // Copy data into dataArray
    memcpy(dataArray, data, dataSize);

    // Escape magic numbers within the dataArray
    escapeMagicNumbers(dataArray, dataSize, maxDataSize, MAGIC_NUMBER, sizeof(MAGIC_NUMBER));

    // Prepare the packet
    size_t packetSize = dataSize + 8; // Adjusted size of the packet
    byte packet[packetSize];

    // Add magic number
    memcpy(packet, MAGIC_NUMBER, 4);
    // Add packet number
    memcpy(packet + 4, &packetNumber, 2);
    // Add category and data size
    packet[6] = category;
    packet[7] = dataSize;

    // Add escaped data
    memcpy(packet + 8, dataArray, dataSize);

    // Send the packet
    for (size_t i = 0; i < packetSize; i++)
    {
        commsSerial.write(packet[i]);
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
    Serial.begin(9600);     // begin serial communication
    Serial.println("yes");
    commsSerial.begin(1200); // begin communication with APC220
    Serial.println("yes2");   
    GPSSerial.begin(9600);   // begin communication with GPS
    Wire.begin();                 // begin communication with I2C sensors
    //gyro
    if (gyro.init())
    {
        Serial.println("gyro found");
        Serial.println("Position the gyro flat and don't move it - calibrating...");
        smartDelay(1000);
        gyro.autoOffsets();
        Serial.println("Done!");
        gyro.setAccRange(MPU6500_ACC_RANGE_8G); // set g range to 16G, options are 2G, 4G, 8G, 16G. Maybe tweak this later
        Serial.println("Gyro initialized");
    }
    else
    {
        Serial.println("Gyro not found, skipping gyro init");
    }    
    smartDelay(1000);
    // temp/pres sensor
    if (!bmp.begin())
    {
        Serial.println("temp/pres sensor not found, skipping bmp280 init");
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
    packetNumber++;

    float temp = bmp.readTemperature();
    sendPacket(&temp, 1, TEMPERATURE, packetNumber);

    float pressure = bmp.readPressure();
    sendPacket(&pressure, 1, PRESSURE, packetNumber);

    float angles[3] = {gyro.getAngles().x, gyro.getAngles().y, gyro.getAngles().z};
    sendPacket(angles, 3, ROTATION, packetNumber);

    float accel[3] = {gyro.getGValues().x, gyro.getGValues().y, gyro.getGValues().z};
    sendPacket(accel, 3, G_FORCES, packetNumber);

    float uv = UVSensor.getUV();
    sendPacket(&uv, 1, UV_RADIATON, packetNumber);

    float dust = getDustDensity();
    sendPacket(&dust, 1, DUST_CONCENTRATION, packetNumber);

    float gpsPos[3] = {gps.location.lat(), gps.location.lng(), gps.altitude.meters()};
    sendPacket(gpsPos, 3, GPS_POS, packetNumber);

    uint32_t time = millis();
    sendPacket(&time, 1, TIME, packetNumber);

    uint32_t age = gps.location.age();
    sendPacket(&age, 1, GPS_FIX_AGE, packetNumber);

    float hdop = gps.hdop.hdop();
    sendPacket(&hdop, 1, GPS_HDOP, packetNumber);

    uint8_t sats = gps.satellites.value();
    sendPacket(&sats, 1, GPS_NUM_OF_SATS, packetNumber);

    unsigned long passed = gps.passedChecksum();
    unsigned long failed = gps.failedChecksum();
    unsigned long total = passed + failed;
    float failPercentage = (failed / (float)total) * 100.0;
    sendPacket(&failPercentage, 1, GPS_FAIL_PERCENTAGE, packetNumber);

    if (millis() > 5000 && gps.charsProcessed() < 10)
    {
        Serial.println(F("No GPS data received: check wiring"));
    }
    else
    {
        smartDelay(1000);
    }
}
