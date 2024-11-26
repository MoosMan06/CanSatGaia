/*
Wirering:
APC220 > Arduino
gnd > gnd
vcc > 5V
en > Not connected
RXD > D3
TXD > D2
AUX > Not connected
SET > Not connected
*/


#include <SoftwareSerial.h>

SoftwareSerial mySerial(2, 3); 

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);
}

void loop() {
  if(mySerial.available() > 1){
    String input = mySerial.readString();
    Serial.println(input);    
  }
  delay(20);
}
