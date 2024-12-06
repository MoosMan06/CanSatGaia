#include <SoftwareSerial.h>

//apc220 config: 450000 2400 9 9600 0

SoftwareSerial mySerial(10, 11);

int number = 0;

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);
}

void loop() {
  number++;
  String message = "APC220 test " + String(number);
  Serial.println(message);
  mySerial.println(message);
  delay(1000); //dies when set lower than 1s, idk why. Try increasing data transfer rate in setup
}
