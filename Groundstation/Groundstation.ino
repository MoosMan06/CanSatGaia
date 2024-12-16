//apc220 config: 450000 2400 9 9600 0

#include <Arduino.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(10, 11); // RX, TX

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);
}

void loop() {
  if (mySerial.available()) {
    byte receivedByte = mySerial.read();
    Serial.print(receivedByte, HEX);
    Serial.print(",");
  }
}
