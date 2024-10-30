/**
 * @file OpenLoop.ino
 * @author Javi (Javier@musotoku.com)
 * @brief
 * @version 0.1
 * @date 2023-05-18
 *
 * @copyright Copyright (c) 2023
 *
 */

const uint16_t C_PIN_UL = 3;
const uint16_t C_PIN_UH = 4;
const uint16_t C_PIN_VL = 5;
const uint16_t C_PIN_VH = 6;
const uint16_t C_PIN_WL = 7;
const uint16_t C_PIN_WH = 8;

void setup()
{
    pinMode(C_PIN_UL, OUTPUT);
    pinMode(C_PIN_UH, OUTPUT);
    pinMode(C_PIN_VL, OUTPUT);
    pinMode(C_PIN_VH, OUTPUT);
    pinMode(C_PIN_WL, OUTPUT);
    pinMode(C_PIN_WH, OUTPUT);
}

void loop()
{
    for (uint16_t i = 0; i < 6; i++)
    {
        switch (i)
        {
        case 0:
            digitalWrite(C_PIN_UH, HIGH);
            digitalWrite(C_PIN_WH, LOW);
            break;
        case 1:
            digitalWrite(C_PIN_WL, HIGH);
            digitalWrite(C_PIN_VL, LOW);
            break;
        case 2:
            digitalWrite(C_PIN_VH, HIGH);
            digitalWrite(C_PIN_UH, LOW);
            break;
        case 3:
            digitalWrite(C_PIN_UL, HIGH);
            digitalWrite(C_PIN_WL, LOW);
            break;
        case 4:
            digitalWrite(C_PIN_WH, HIGH);
            digitalWrite(C_PIN_VH, LOW);
            break;
        case 5:
            digitalWrite(C_PIN_VL, HIGH);
            digitalWrite(C_PIN_UL, LOW);

            break;

        default:
            break;
        }
        delayMicroseconds(5000);
    }
}