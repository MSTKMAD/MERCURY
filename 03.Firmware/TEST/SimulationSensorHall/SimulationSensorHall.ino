/**
 * @file Simulation Sensor Hall.ino
 * @author Javi (Javier@musotoku.com)
 * @brief 
 * @version 1
 * @date 2022-09-01
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "Arduino.h"
const bool CT_CLOCK_WISE = 0;
const bool CT_COUNTER_CLOCK_WISE = 1;
const bool CT_ON = 1;
const bool CT_OFF = 0;

const int PIN_SENSOR_A = 12;
const int PIN_SENSOR_B = 11;
const int PIN_SENSOR_C = 10;
const int PIN_START_STOP = 3;
const int PIN_DIR = 7;
const int PIN_SPEED_CONTROL = A0;

const int DESIRE_MOTOR_FREQ = 200; //Hz
const int NUM_POLES = 4;
const int NUM_SENSOR = 3;
const int TOP_FREQ_SENSOR_PWM = DESIRE_MOTOR_FREQ * NUM_POLES * NUM_SENSOR;

const int SAFE_PERC = 100 + 20; // %

bool dir = CT_CLOCK_WISE;
bool butt_dir;
bool prev_butt_dir;
bool prev_start_stop;
bool start_stop;
bool status;

bool LUT_clockwise_sensor_A[] = {0, 0, 1, 1, 1, 0};
bool LUT_clockwise_sensor_B[] = {0, 0, 0, 1, 1, 1};
bool LUT_clockwise_sensor_C[] = {1, 0, 0, 0, 1, 1};

int step = 0;
int32_t t0 = 0;
int32_t t1 = 0;
int32_t frequency = 1000;                       // Hz
int32_t period = 1000000 / (frequency * 2 * 3); // us

void setup()
{
    Serial.begin(9600);
    pinMode(PIN_SENSOR_A, OUTPUT);
    pinMode(PIN_SENSOR_B, OUTPUT);
    pinMode(PIN_SENSOR_C, OUTPUT);
    pinMode(PIN_START_STOP, INPUT);
    pinMode(PIN_SPEED_CONTROL, INPUT);
    pinMode(PIN_DIR, INPUT);
}

void loop()
{
    while (status == CT_ON)

    {
        digitalWrite(PIN_SENSOR_A, LUT_clockwise_sensor_A[step]);
        digitalWrite(PIN_SENSOR_B, LUT_clockwise_sensor_B[step]);
        digitalWrite(PIN_SENSOR_C, LUT_clockwise_sensor_C[step]);
        if (dir == CT_CLOCK_WISE)
        {
            step++;
            if (step > 5)
            {
                step = 0;
            }
        }
        else
        {
            step--;
            if (step < 0)
            {
                step = 5;
            }
        }
        analogReadResolution(12);
        frequency = analogRead(PIN_SPEED_CONTROL);
        // Serial.print(frequency);
        frequency = frequency * TOP_FREQ_SENSOR_PWM * SAFE_PERC / 100 / 4096;
        // Serial.print("/");
        // Serial.print(frequency);
        //Serial.print("/");
        period = 1000000 / (frequency * 6);
        //Serial.println(period);
        //t0 = micros();
        start_stop = debounce_START_STOP();
        if ((prev_start_stop == LOW) && (start_stop == HIGH))
        {
            status = CT_OFF;
        }
        prev_start_stop = start_stop;

        butt_dir = debounce_DIR();
        if ((prev_butt_dir == LOW) && (butt_dir == HIGH))
        {
            dir = !dir;
        }
        prev_butt_dir = butt_dir;
        //t1 = micros();
        //period = period - (t1 - t0);
        delayMicroseconds(period);
    }
    while (status == CT_OFF)
    {
        start_stop = debounce_START_STOP();
        if ((prev_start_stop == LOW) && (start_stop == HIGH))
        {
            status = CT_ON;
        }
        prev_start_stop = start_stop;
    }
}

/* function debounce_DIR */
bool debounce_DIR()
{
    int reading = digitalRead(PIN_DIR);
    static bool nowStatus = false;
    static bool oldStatus = false;
    static unsigned long lastDebounceTime = 0;
    unsigned long debounceDelay = 50;
    if (reading != oldStatus)
    {
        lastDebounceTime = millis();
    }
    if ((millis() - lastDebounceTime) > debounceDelay)
    {
        if (reading != nowStatus)
        {
            nowStatus = reading;
        }
    }
    oldStatus = reading;
    return nowStatus;
}
/* function debounce_START_STOP */
bool debounce_START_STOP()
{
    int reading = digitalRead(PIN_START_STOP);
    static bool nowStatus = false;
    static bool oldStatus = false;
    static unsigned long lastDebounceTime = 0;
    unsigned long debounceDelay = 50;
    if (reading != oldStatus)
    {
        lastDebounceTime = millis();
    }
    if ((millis() - lastDebounceTime) > debounceDelay)
    {
        if (reading != nowStatus)
        {
            nowStatus = reading;
        }
    }
    oldStatus = reading;
    return nowStatus;
}
