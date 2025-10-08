// archivo que pruebe una pantalla oled  ssd1306 con un display 0.96 pulgadas por SPI
#include "Adafruit_SSD1306.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
const uint16_t C_PIN_DC = 11;
const uint16_t C_PIN_SS_LED = 17;
const uint16_t C_PIN_SCK = 18;
const uint16_t C_PIN_MOSI = 19;
const uint16_t C_PIN_DSP_RST = 22;

Adafruit_SSD1306 display(128, 64, &SPI, C_PIN_DC, C_PIN_DSP_RST, C_PIN_SS_LED);

void setup() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("Hello, World!");
  display.display();
}

void loop() {
  // No se necesita hacer nada en el loop
}