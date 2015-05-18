//      |---------------------------------------------------------------------------|
//      |            The hardware connections are as follows:                       |
//      |--------------------------------|------------------------------------------|
//      |  Connector at Olimexino-328    |      Connector at BB-VNH3SP30            |
//      |--------------------------------|------------------------------------------|
//      |    Not connected!              |          CTRL<1>,  VIN                   |
//      |    Power<3>, 5V         	 |          CTRL<2>,  +5V                   |
//      |    Power<4>, GND               |          CTRL<3>,  GND                   |
//      |    Digital<9> D7               |          CTRL<4>,  INA                   |
//      |    Digital<13> D6              |          CTRL<5>,  INB                   |
//      |    Digital<11> D5              |          CTRL<6>,  PWM                   |
//      |    Digital<10> D3              |          CTRL<7>,  ENA/DIAGA             |
//      |    Digital<12> D2              |          CTRL<8>,  ENB/DIAGB             |
//      |--------------------------------|------------------------------------------|

// 24C16WP 
// 24C16AN

const byte DEVADDR1 = 0x50; // 2048 Bytes
const byte DEVADDR2 = 0x54; // 2048 Bytes

#include <Wire.h>

const int inaPin = 6;    //D6
const int inbPin = 7;    //D7
const int pwmPin = 5;    //D5
const int diagaPin = 3;  //D3
const int diagbPin = 2;  //D2

int i = 0;

void setup() {

  
  Wire.begin();

  Serial.begin(9600);
  
  pinMode(inaPin, OUTPUT);
  pinMode(inbPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);

  pinMode(diagaPin, INPUT);
  pinMode(diagbPin, INPUT);
  
  pinMode(13,OUTPUT);
  pinMode(A1,OUTPUT);
  
 
}

void loop() {

  // i2c();
  
  // Стоп

  digitalWrite(inaPin, LOW);
  digitalWrite(inbPin, LOW);
  delay(500);
  
  delay(5000);
  
  // По часовой
  digitalWrite(inaPin, HIGH);
  digitalWrite(inbPin, LOW);
  analogWrite(pwmPin, 200);
  digitalWrite(13,HIGH);
  digitalWrite(A1,LOW);
  
  delay(1500);
  
  //STOP
  digitalWrite(inaPin, LOW);
  digitalWrite(inbPin, LOW);
  delay(500);
  
  // Против часовой
  digitalWrite(inaPin, LOW);
  digitalWrite(inbPin, HIGH);
  analogWrite(pwmPin, 200);
  
  digitalWrite(13,LOW);
  digitalWrite(A1,HIGH);
  
  delay(1500);
  
   // Стоп
  digitalWrite(inaPin, HIGH);
  digitalWrite(inbPin, HIGH);
  delay(500);
  
}

void i2c() {
  
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknow error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(5000);           // wait 5 seconds for next scan
}
