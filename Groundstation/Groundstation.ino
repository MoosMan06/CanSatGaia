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

char message;

void setup() {

  Serial.begin(9600);
  mySerial.begin(9600);
}

void loop() {

  message = mySerial.listen();
  Serial.println(message);
  delay(1);
}
