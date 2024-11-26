#include <SoftwareSerial.h>

/* plug in apc220 at pin 8-GND with apc pcb pointing away from Arduino
*/

const int vrs = 101;
const int rxPin = 10; //rx pin is tx pin on apc
const int txPin = 11; //tx pin is rx pin on apc
const int setPin = 8; //set pin
const int enPin = 12; // no use
const int fiveV = 13;  // 5V to the APC220

int echo_mode = 0; 

#define TIMEOUT 60000L
#define PING_INTERVAL 2000L

unsigned long timeout,lastPing;

#define PARM_LGT 10
#define CMD_BUF_LGT 50

char var1[PARM_LGT], var2[PARM_LGT], var3[PARM_LGT], var4[PARM_LGT], var5[PARM_LGT], var6[PARM_LGT];

SoftwareSerial apc220(rxPin, txPin); // Crt softserial port and bind tx/rx to appropriate PINS

void set_para(char hz[], char rf_rate[], char pwr[], char uart_rate[], char sc[]) {
// sanity chk
  if (strlen(hz) != 6) {
    Serial.println("Freq parm not 6 digits... - legal is 418000 - 455000");
  return;
  }
  if (strlen(rf_rate) != 1 ) {
    Serial.println("RF parm is not 1 digit: legal values is 1/2/3/4");
    return;
  }
  if (strlen(pwr) != 1 ) {
    Serial.println("Power parm is not 1 digit: legal values is 1..9"); 
    return;
  }

  if (strlen(uart_rate) != 1 ) {
    Serial.println("Uart baudrate parm is not 1 digit: legal values is 0..6"); 
    return;
  }
    if (strlen(sc) != 1 ) {
    Serial.println("Parity parm is not 1 digit: legal values is 0/1/2"); 
    return;
  }


  digitalWrite(setPin, LOW); // set radio in config mode
  delay(50);
  apc220.print("WR");
  apc220.print(" ");
  
  apc220.print(hz);
  apc220.print(" ");
  
  apc220.print(rf_rate);
  apc220.print(" ");
  
  apc220.print(pwr);
  apc220.print(" ");
  
  apc220.print(uart_rate);
  apc220.print(" ");
  
  apc220.print(sc);
  
  apc220.write(0x0D);
  apc220.write(0x0A);
  delay(10);

   // read feedback from radio	
 
  while (apc220.available()) {
    Serial.write(apc220.read());
  }

  digitalWrite(setPin, HIGH); // set radio back in normal rx/tx mode
}

void get_para(void) {
  int i = 0;
  char buff[CMD_BUF_LGT];

  digitalWrite(setPin, LOW); // set radio in config mode
  
  delay(10); // stabilize please
  
  apc220.println("RD"); // ask for data
  delay(10);
  
  while (apc220.available()) {
    Serial.write(apc220.read()); 
  }
  
  digitalWrite(setPin, HIGH); // set radio back in normal tx/rx mode
}

void resetPing(void)  {
  echo_mode = 0;
  timeout = millis();  
}

void helpMenu(void) {
  Serial.println("");
  Serial.println("commands:");
  Serial.println("  r :  Read apc220 radio config"); 
  Serial.println("  e :  go into echo mode: receive char, add 1 and return");
  Serial.println("  n :  no more echo - back to normal"); 
  Serial.println("  w :  Write apc radio config ..."); 
  Serial.println("     'W'  FFFFFF R P B C - number of letters indicates precise number of digits" );
  Serial.println("      FFFFFF: frequency: 434000 (434 MHz) range 418000-455000 "); 
  Serial.println("      R:      Rf data rate       - 1/2/3/4 equals 2400/4800/9600/19200bps");
  Serial.println("      P:      Radio output power - 0 .. 9 9 equals 13dBm(20mW).");
  Serial.println("      B:      UART baudrate      - 0/1/2/3/4/5/6 equals 1200/2400/4800/9600/19200/38400/57600bps");
  Serial.println("      C:      Byte Chek Parity   - 0/1/2 equals NoCheck(8N1)/EvenParity(8E1)/OddParity(8O1)"); 
  Serial.println("");
  Serial.println("Write example: w 434000 3 9 3 0 is...");
  Serial.println("    434,000 MHz 9600 baud in air, 20mW, 9600baud on UART, No Parity(8N1)");
  Serial.println("After 30 seconds with no keyboard input we will emit a char every two second");
}

void get_cmd(void) {

  if ((echo_mode == 0) && (TIMEOUT < (millis() - timeout))) { // time and state for beacon mode ?
    apc220.write('!');
    Serial.println("going into ping mode - emit a char every two second on apc220  !");    
    lastPing = millis();
    echo_mode = 2;
    goto xxx;
  }
  
  if (echo_mode == 2) { // beacon mode
    if (PING_INTERVAL < (millis() - lastPing)) {
      apc220.write('x');
      Serial.println(lastPing/1000);       
      lastPing = millis();
    }
    goto xxx;
  }

  if (echo_mode == 1) { // echo mode
    char c;
    
     if (apc220.available()) {
        c = apc220.read();
        Serial.write(c); 
        apc220.write(c+1); 
      }
      goto xxx;
  }
  
  xxx:
 
   if (Serial.available()) {  
    int i=0;
    char buff[CMD_BUF_LGT];
   
   delay(100);
   
    while (Serial.available() && (i < CMD_BUF_LGT-1)) {
      buff[i++] = Serial.read();
    }
    buff[i] = '\0';
    Serial.println(buff); //echo input
 
    resetPing();

    var1[0] = 0x00; // reset
    sscanf(buff, "%s %s %s %s %s %s", var1, var2, var3, var4, var5, var6);
    switch (var1[0]) { // one letter commands :-)
    case 'r': {
      get_para();
      break;
    }
    case 'w': {
      set_para(var2, var3, var4, var5, var6);
      break;
    }
    case 'e' :{
      echo_mode = 1;
      break;
    }
    case 'n' :{
      echo_mode = 0;
      break;
    }
    default:
      echo_mode = 0;    
      helpMenu();
    }
   }
}

void setupSoftAPC(void)
{
  pinMode(setPin,OUTPUT);
  digitalWrite(setPin,HIGH);
  pinMode(fiveV,OUTPUT);  // 5V
  digitalWrite(fiveV,HIGH); // turn on 5V
  delay(50);
  pinMode(enPin,OUTPUT); // ENABLE
  digitalWrite(enPin,HIGH); //
  delay(100);
  apc220.begin(9600);
}

void setup() {
   Serial.begin(9600);
   Serial.print("APC version: ");
   Serial.println(vrs);
   Serial.println(__DATE__);
   setupSoftAPC();
   timeout = millis(); // just to start
   helpMenu();
}

void loop() {
	get_cmd();
}

