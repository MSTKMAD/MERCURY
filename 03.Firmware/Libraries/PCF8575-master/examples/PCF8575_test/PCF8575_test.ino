//
//    FILE: PCF8575_test.ino
//  AUTHOR: Rob Tillaart
//    DATE: 2020-07-20
// PUPROSE: test PCF8575 library
//     URL: https://github.com/RobTillaart/PCF8575


#include "PCF8575.h"

PCF8575 PCF(0x20);


void setup()
{
  Serial.begin(115200);
  Serial.println(__FILE__);
  Serial.print("PCF8575_test version: ");
  Serial.println(PCF8575_LIB_VERSION);

  PCF.begin();

  uint16_t x = PCF.read16();
  Serial.print("Read ");
  printHex(x);
  delay(1000);
}


void loop()
{
  Serial.print(PCF.read(0));
  Serial.print("/");
  Serial.print(PCF.read(1));
  Serial.print("/");
  Serial.print(PCF.read(2));
  Serial.print("/");
  Serial.print(PCF.read(3));
  Serial.print("/");
  Serial.print(PCF.read(4));
  Serial.print("/");
  Serial.print(PCF.read(5));
  Serial.print("/");
  Serial.print(PCF.read(6));
  Serial.print("/");
  Serial.print(PCF.read(7));
  Serial.print("/");
  Serial.print(PCF.read(8));
  Serial.print("/");
  Serial.print(PCF.read(9));
  Serial.print("/");
  Serial.print(PCF.read(10));
  Serial.print("/");
  Serial.print(PCF.read(11));
  Serial.print("/");
  Serial.print(PCF.read(12));
  Serial.print("/");
  Serial.print(PCF.read(13));
  Serial.print("/");
  Serial.print(PCF.read(14));
  Serial.print("/");
  Serial.println(PCF.read(15));
  delay(1000);
  // Serial.println("HLT");
  // while (Serial.available() == 0);
  // switch(Serial.read())
  // {
  //   case 'H': doHigh(); break;
  //   case 'L': doLow(); break;
  //   case 'T': doToggle(); break;
  // }
}


void doHigh()
{
  PCF.write(4, HIGH);
  int x = PCF.read16();
  Serial.print("Read ");
  printHex(x);
}


void doLow()
{
  PCF.write(4, LOW);
  int x = PCF.read16();
  Serial.print("Read ");
  printHex(x);
}


void doToggle()
{
  PCF.toggle(4);
  int x = PCF.read16();
  Serial.print("Read ");
  printHex(x);
}


void printHex(uint16_t x)
{
  if (x < 0x1000) Serial.print('0');
  if (x < 0x100)  Serial.print('0');
  if (x < 0x10)   Serial.print('0');
  Serial.println(x, HEX);
}


// -- END OF FILE --

