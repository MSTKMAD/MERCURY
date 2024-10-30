/**
 * @file TestBenchMotorBrushed.ino
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-10-16
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "Arduino.h"
#include "Wire.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"
#include <MilliTimer.h>
#include "RP2040_PWM.h"
#include <pico/multicore.h>

const int PIN_PWM_0 = 14;
const int PIN_PWM_1 = 15;

const int PIN_RST = 17;
const int PIN_MISO = 20;
const int PIN_SS = 21;
const int PIN_SCK = 18;
const int PIN_MOSI = 19;

const int DEFAULT_FREQ_PWM = 10000;
const int DEFAULT_DUTY_CYCLE = 70;

RP2040_PWM *PWM_Instance[2];
Adafruit_SSD1306 display(128, 64, PIN_MOSI, PIN_SCK, PIN_MISO, PIN_RST, PIN_SS);

void setup()
{
    Serial.begin(115200);

    pinMode(PIN_PWM_0, OUTPUT);
    pinMode(PIN_PWM_1, OUTPUT);

    PWM_Instance[0] = new RP2040_PWM(PIN_PWM_0, DEFAULT_FREQ_PWM, DEFAULT_DUTY_CYCLE);
    PWM_Instance[1] = new RP2040_PWM(PIN_PWM_1, DEFAULT_FREQ_PWM, DEFAULT_DUTY_CYCLE);

    display.begin();
    display.clearDisplay();

    display.setTextSize(2);

    PWM_Instance[0]->setPWM(PIN_PWM_0, DEFAULT_FREQ_PWM, 50, true);
}

void loop()
{
}