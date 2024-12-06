#include <SoftwareSerial.h>

//apc220 config: 450000 2400 9 9600 0

SoftwareSerial mySerial(10, 11);

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);
}

void loop() {
  if (mySerial.available()) {
    String input = mySerial.readString();
    Serial.println(input);
  }
  delay(1);
}
