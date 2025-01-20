// apc220 config: w 432600 1 9 4 0

#include <Arduino.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(10, 11); // RX, TX

void setup() {
  Serial.begin(19200);
  mySerial.begin(19200);
  Serial.println("groundstation startup succes");
}

void loop() {
  if (mySerial.available()) {
    Serial.print("received: ");
    byte receivedByte = mySerial.read();
    Serial.print(receivedByte, HEX);
    Serial.println(",");
  }
}
