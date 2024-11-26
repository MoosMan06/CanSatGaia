/*
 gnd > gnd
 VCC > 5V
 EN > Disconnected
 RXD > D10
 TXD > D9
 AUX > Disconnected
 SET > Disconnected
*/

#include <SoftwareSerial.h>
SoftwareSerial mySerial(9, 10);

int number = 0;

void setup() {

  Serial.begin(9600);
  mySerial.begin(9600);
}

void loop() {

  (number++);
  Serial.print("APC220 sent   ");
  Serial.println(number);
  mySerial.print("APC220 received   ");
  mySerial.println(number);
  delay(1000);
}
