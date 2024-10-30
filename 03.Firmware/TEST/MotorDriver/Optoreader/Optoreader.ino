
#include "MilliTimer.h"

const uint16_t C_PIN_OPTO = 5;

uint32_t time_0 = 0;
uint32_t time_1 = 0;
uint32_t period = 0;

uint32_t indx = 0;

bool new_state;
bool old_state;

MilliTimer timer;
uint32_t buffer[24];
void setup()
{
    // put your setup code here, to run once:

    Serial.begin(115200);
    time_0 = micros();
}

void loop()
{

    new_state = digitalRead(C_PIN_OPTO);
    if ((new_state == HIGH) && (old_state == LOW))
    {
        time_1 = micros();
        period = time_1 - time_0;
        buffer[indx] = period;
        indx++;
        if (indx == 24)
        {
            for (int i = 0; i < 24; i++)
            {
                Serial.printf("/*%d*/\n", buffer[i]);
            }
            indx = 0;
        }
        time_0 = time_1;
    }
    old_state = new_state;
}
