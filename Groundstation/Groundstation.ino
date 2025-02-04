#include <Arduino.h>
#include <SoftwareSerial.h>

// apc220 config: w 432600 1 9 0 0

SoftwareSerial commsSerial(10, 11); // RX, TX

void setup() {
  Serial.begin(9600);
  commsSerial.begin(1200);
}

void loop() {
  if (commsSerial.available()) {
    byte receivedByte = commsSerial.read();
    Serial.write(receivedByte);
  }
}
