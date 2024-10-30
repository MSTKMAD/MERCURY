#include <Arduino.h>
#include <U8g2lib.h>
#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

//U8G2_SSD1306_72X40_ER_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
U8G2_SSD1306_72X40_ER_F_4W_SW_SPI u8g2(U8G2_R0, /* clock=*/ 13, /* data=*/ 11, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);
float num = 10.5;
// font = u8g2_font_battery24_tr;
// font = u8g2_font_freedoomr25_tu;
void setup()
{
  // put your setup code here, to run once:
  u8g2.begin();
}

void loop()
{
  // put your main code here, to run repeatedly:
  //u8g2.clearBuffer();
  //u8g2.setFontMode(1);
  //u8g2.setFont(u8g2_font_freedoomr25_mn);
  //u8g2.setCursor(0, 40);
  //u8g2.print("10,5");
  //u8g2.sendBuffer();
  delay(250);
}
