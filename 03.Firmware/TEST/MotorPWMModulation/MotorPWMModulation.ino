/**
 * @file MotorPWMModulation.ino
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-03-06
 *
 * @copyright Copyright (c) 2024
 *
 */

const int PIN_IN_1 = 10;
const int PIN_IN_2 = 5;

const int PIN_GEN = 13;

bool single = false;

void setup()
{
    pinMode(PIN_GEN, INPUT);
    pinMode(PIN_IN_1, OUTPUT);
    pinMode(PIN_IN_2, OUTPUT);
}

void loop()
{
    if (digitalRead(PIN_GEN))
    {
        digitalWrite(PIN_IN_1, HIGH);
        digitalWrite(PIN_IN_2, LOW);
        single = false;
    }
    else
    {
        if (!single)
        {
            digitalWrite(PIN_IN_1, LOW);
            delay(1);
            digitalWrite(PIN_IN_2, HIGH);
            delay(1);
            digitalWrite(PIN_IN_2, LOW);
            delay(1);
            single = true;
        }
    }
}