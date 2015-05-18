// Arduino 1.0.6
// 
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

#include <Wire.h>

const byte EEPROM_ADDR1 = 0x50; // 2048 Bytes
const byte EEPROM_ADDR2 = 0x54; // 2048 Bytes

const int LED1 = 13; // Default LED
const int LED2 = A1; // Romik LED

const int inaPin = 6;    //D6
const int inbPin = 7;    //D7
const int pwmPin = 5;    //D5
const int diagaPin = 3;  //D3
const int diagbPin = 2;  //D2

int i = 0;

unsigned long currentMillis;
unsigned long PreviousInterval = 0;        // Для всех внутренних функций

// ----------------------------------- Setup ---------------------------------------------------

void setup() {

  Wire.begin();


  Serial.begin(9600);

  pinMode(inaPin, OUTPUT);
  pinMode(inbPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);

  pinMode(diagaPin, INPUT);
  pinMode(diagbPin, INPUT);

  pinMode(LED1,OUTPUT);
  pinMode(LED2,OUTPUT);

  long sum =0;

 Serial.println("Start Write");
 
  for (int addr=0;addr<4096;addr++) {
    dual_write(addr,0x1);   
  }

Serial.println("Start Read");
 
 for (int addr=0;addr<4096;addr++) {
   sum = sum + dual_read(addr);
 }
 
 Serial.println(sum);

}

// ----------------------------------------- LOOP MAIN ----------------------------------------

void loop() {

  // i2c();

  // Стоп


  currentMillis = millis();

  if(currentMillis - PreviousInterval > 1000) {
    PreviousInterval = currentMillis;  
    if (digitalRead(LED1) == HIGH) 
      digitalWrite(LED1,LOW);
    else digitalWrite(LED1,HIGH);

    if (digitalRead(LED2) == HIGH) 
      digitalWrite(LED2,LOW);
    else digitalWrite(LED2,HIGH);

  }

}

// -------------------------------------- Functions -----------------------------------------

void motor( void ) {


  digitalWrite(inaPin, LOW);
  digitalWrite(inbPin, LOW);
  delay(500);

  delay(5000);

  // По часовой
  digitalWrite(inaPin, HIGH);
  digitalWrite(inbPin, LOW);
  analogWrite(pwmPin, 200);
  delay(1500);

  //STOP
  digitalWrite(inaPin, LOW);
  digitalWrite(inbPin, LOW);
  delay(500);

  // Против часовой
  digitalWrite(inaPin, LOW);
  digitalWrite(inbPin, HIGH);
  analogWrite(pwmPin, 200);
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


char dual_read(unsigned eeaddress) {

  unsigned addr;
  byte devaddr;
  
  if (eeaddress < 2048) {
    addr = eeaddress;
    devaddr = EEPROM_ADDR1; 
  } else {
    addr = eeaddress - 2048;
    devaddr = EEPROM_ADDR2;
  }
  
  char out = eeprom_read_byte(devaddr,addr);   

  return(out);
  
}

void dual_write(unsigned eeaddress, char data) {
  
  unsigned addr;
  byte devaddr;
  
  if (eeaddress < 2048) {
    addr = eeaddress;
    devaddr = EEPROM_ADDR1; 
  } else {
    addr = eeaddress - 2048;
    devaddr = EEPROM_ADDR2;
  }
  
  eeprom_write_byte(devaddr,addr,data);   

}


char eeprom_read_byte(byte deviceaddress, unsigned eeaddr)
{
    byte rdata = -1;

    // Three lsb of Device address byte are bits 8-10 of eeaddress
    byte devaddr = deviceaddress | ((eeaddr >> 8) & 0x07);
    byte addr    = eeaddr;

    Wire.beginTransmission(devaddr);
    Wire.write(int(addr));
    Wire.endTransmission();
    Wire.requestFrom(int(devaddr), 1);
    if (Wire.available()) {
        rdata = Wire.read();
    }
    return rdata;
}

void eeprom_write_byte(byte deviceaddress, int eeaddress, char data)
{
    // Three lsb of Device address byte are bits 8-10 of eeaddress
    byte devaddr = deviceaddress | ((eeaddress >> 8) & 0x07);
    byte addr    = eeaddress;
    Wire.beginTransmission(devaddr);
    Wire.write(int(addr));
    Wire.write(char(data));
    Wire.endTransmission();
    delay(10);
}

