/**
 * @file MotorDriver.ino
 * @author Javi (Javier@musotoku.com)
 * @brief 
 * @version 1
 * @date 2022-09-09
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "Arduino.h"
#include "Wire.h"
#include "PCF8575.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"
#include <MilliTimer.h>
#include "RP2040_PWM.h"
#include <pico/multicore.h>

#define FIX_MODE 0
#define TOP_MODE 1
#define BLOQ_MODE 2
//*******************************************************************//
//                      Functions & Classes                          //
//*******************************************************************//
const int PIN_HALL_A = 12;
const int PIN_HALL_B = 13;
const int PIN_HALL_C = 14;

const int PIN_PWM_0 = 8;
const int PIN_PWM_1 = 10;
const int PIN_PWM_2 = 2;
const int PIN_PWM_3 = 6;
const int PIN_PWM_4 = 22;
const int PIN_PWM_5 = 20;

const int PIN_EN_0 = 9;
const int PIN_EN_1 = 11;
const int PIN_EN_2 = 3;
const int PIN_EN_3 = 7;
const int PIN_EN_4 = 21;
const int PIN_EN_5 = 1;

const int PIN_ON_OFF = 0;
const int PIN_SPD_UP = 1;
const int PIN_SPD_DOWN = 2;
const int PIN_FRC_UP = 3;
const int PIN_FRC_DOWN = 4;

const int PIN_SLOW_TRIGGER = 5;
const int PIN_FAST_TRIGGER = 0;

const int PIN_RST = 15;
const int PIN_MISO = 16;
const int PIN_SS = 17;
const int PIN_SCK = 18;
const int PIN_MOSI = 19;

const int PIN_SDA = 4;
const int PIN_SCL = 5;

const int PIN_SENSOR_IR_1 = A0;
const int PIN_SENSOR_IR_2 = A1;
const int PIN_ISENSE = A2;

const int CLOCKWISE[] = {1, 0, 4, 6, 7, 3};

const int NUM_OF_PWM = 12;

const int DEFAULT_FREQ_PWM = 53600;
const int DEFAULT_DUTY_CYCLE = 70;

const int PWM_0[] = {0, 0, 0, 1, 1, 0};
const int PWM_1[] = {1, 1, 0, 0, 0, 0};
const int PWM_2[] = {0, 1, 1, 0, 0, 0};
const int PWM_3[] = {0, 0, 0, 0, 1, 1};
const int PWM_4[] = {1, 0, 0, 0, 0, 1};
const int PWM_5[] = {0, 0, 1, 1, 0, 0};

const int EN_0[] = {0, 0, 0, 0, 1, 1, 1, 0};
const int EN_1[] = {1, 1, 1, 0, 0, 0, 0, 0};
const int EN_2[] = {0, 1, 1, 1, 0, 0, 0, 0};
const int EN_3[] = {0, 0, 0, 0, 0, 1, 1, 1};
const int EN_4[] = {1, 0, 0, 0, 0, 0, 0, 1};
const int EN_5[] = {0, 0, 0, 1, 1, 0, 0, 0};

const int NUM_BLOQ = 4;
const int DUTY_BLOQ[NUM_BLOQ] = {0, 25, 50, 75};
int bloq = 0;

RP2040_PWM *PWM_Instance[NUM_OF_PWM];

const int CONT_PULSE = 20;

int status;
int next_status;
int sensor;

bool oldHallA;
bool newHallA;

bool measure_ok;

bool start = false;
int address_I2C_extra = 0x20;

bool hall_a;
bool hall_b;
bool hall_c;

uint32_t taco_0;
uint32_t taco_1;
uint32_t motor_speed;

int cont_hall_pulses = 0;
int hall_period = 0;

int control_mode = TOP_MODE;

PCF8575 pcf8575(address_I2C_extra, PIN_SDA, PIN_SCL);
Adafruit_SSD1306 display(128, 64, PIN_MOSI, PIN_SCK, PIN_MISO, PIN_RST, PIN_SS);

class Button
{
private:
    int _pin;
    bool read;
    bool _nowStatus;
    bool _oldStatus;
    unsigned long _lastDebounceTime;
    unsigned long _debounceDelay;

public:
    Button(int pin, int debounceDelayms = 50)
    {
        _debounceDelay = debounceDelayms;
        _pin = pin;
        _nowStatus = digitalRead(_pin);
        _lastDebounceTime = millis();
    }
    bool ReadState()
    {
        read = digitalRead(_pin);
        if (read != _oldStatus)
        {
            _lastDebounceTime = millis();
        }
        if ((millis() - _lastDebounceTime) > _debounceDelay)
        {
            if (read != _nowStatus)
            {
                _nowStatus = read;
            }
        }
        _oldStatus = read;
        return _nowStatus;
    }
};

Button butt_on_off(PIN_ON_OFF);
Button butt_spd_up(PIN_SPD_DOWN);
Button butt_spd_down(PIN_SPD_UP);
Button butt_frc_down(PIN_FRC_DOWN);
Button butt_frc_up(PIN_FRC_UP);

bool act_button_on_off = true;
bool prev_button_on_off = true;
bool act_button_spd_up = true;
bool prev_button_spd_up = true;
bool act_button_spd_down = true;
bool prev_button_spd_down = true;
bool act_button_frc_down = true;
bool prev_button_frc_down = true;
bool act_button_frc_up = true;
bool prev_button_frc_up = true;

MilliTimer refresh_display;

bool sample_button = false;
bool prev_sample_button = false;

void setup()
{
    Serial.begin(115200);
    pcf8575.pinMode(PIN_ON_OFF, OUTPUT);
    pcf8575.pinMode(PIN_SPD_UP, OUTPUT);
    pcf8575.pinMode(PIN_SPD_DOWN, OUTPUT);
    pcf8575.pinMode(PIN_FRC_UP, OUTPUT);
    pcf8575.pinMode(PIN_FRC_DOWN, OUTPUT);
    pcf8575.pinMode(PIN_SLOW_TRIGGER, OUTPUT);

    pinMode(PIN_HALL_A, INPUT);
    pinMode(PIN_HALL_B, INPUT);
    pinMode(PIN_HALL_C, INPUT);

    pinMode(PIN_PWM_0, OUTPUT);
    pinMode(PIN_PWM_1, OUTPUT);
    pinMode(PIN_PWM_2, OUTPUT);
    pinMode(PIN_PWM_3, OUTPUT);
    pinMode(PIN_PWM_4, OUTPUT);
    pinMode(PIN_PWM_5, OUTPUT);

    pinMode(PIN_EN_0, OUTPUT);
    pinMode(PIN_EN_1, OUTPUT);
    pinMode(PIN_EN_2, OUTPUT);
    pinMode(PIN_EN_3, OUTPUT);
    pinMode(PIN_EN_4, OUTPUT);
    pinMode(PIN_EN_5, OUTPUT);

    pinMode(PIN_FAST_TRIGGER, INPUT);

    pinMode(PIN_SENSOR_IR_1, INPUT);
    pinMode(PIN_SENSOR_IR_2, INPUT);
    pinMode(PIN_ISENSE, INPUT);

    PWM_Instance[0] = new RP2040_PWM(PIN_PWM_0, DEFAULT_FREQ_PWM, DEFAULT_DUTY_CYCLE);
    PWM_Instance[1] = new RP2040_PWM(PIN_PWM_1, DEFAULT_FREQ_PWM, DEFAULT_DUTY_CYCLE);
    PWM_Instance[2] = new RP2040_PWM(PIN_PWM_2, DEFAULT_FREQ_PWM, DEFAULT_DUTY_CYCLE);
    PWM_Instance[3] = new RP2040_PWM(PIN_PWM_3, DEFAULT_FREQ_PWM, DEFAULT_DUTY_CYCLE);
    PWM_Instance[4] = new RP2040_PWM(PIN_PWM_4, DEFAULT_FREQ_PWM, DEFAULT_DUTY_CYCLE);
    PWM_Instance[5] = new RP2040_PWM(PIN_PWM_5, DEFAULT_FREQ_PWM, DEFAULT_DUTY_CYCLE);

    digitalWrite(PIN_SS, HIGH);
    display.begin();
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    //display.writeFastHLine(0, 0, 128, WHITE);
    display.setCursor(0, 0);
    display.print("NOW");
    //display.writeFastHLine(0, 21, 128, WHITE);
    display.setCursor(0, 25);
    display.print("OBJ");
    //display.writeFastHLine(0, 42, 128, WHITE);
    display.setCursor(0, 50);
    display.print("FORCE");
    // display.writeFastHLine(0, 63, 128, WHITE);
    //display.fillRect(0, 0, 128, 64, WHITE);

    display.display();
    refresh_display.set(250);

    digitalWrite(PIN_EN_0, HIGH);
    digitalWrite(PIN_EN_1, HIGH);
    digitalWrite(PIN_EN_2, HIGH);
    digitalWrite(PIN_EN_3, HIGH);
    digitalWrite(PIN_EN_4, HIGH);
    digitalWrite(PIN_EN_5, HIGH);

    while (!start)
    {
        hall_a = digitalRead(PIN_HALL_A);
        hall_b = digitalRead(PIN_HALL_B);
        hall_c = digitalRead(PIN_HALL_C);
        oldHallA = newHallA;
        newHallA = hall_a;

        sensor = hall_a << 2 + hall_b << 1 + hall_c;
        for (size_t i = 0; i < 6; i++)
        {
            if (CLOCKWISE[i] == sensor)
            {
                status = i;
                next_status = i + 1;
                if (next_status > 5)
                {
                    next_status = 0;
                }
                start = true;
            }
        }
    }
    multicore_launch_core1(core1_entry);
}

void loop()
{
    /*          Hall sensor Read           */
    hall_a = digitalRead(PIN_HALL_A);
    hall_b = digitalRead(PIN_HALL_B);
    hall_c = digitalRead(PIN_HALL_C);

    newHallA = hall_a;

    //----------------- TACO ----------------------
    if ((oldHallA == LOW) && (newHallA == HIGH))
    {
        if (cont_hall_pulses == 0)
        {
            taco_0 = micros();
        }
    }
    else if ((oldHallA == HIGH) && (newHallA == LOW))
    {
        if (cont_hall_pulses == CONT_PULSE)
        {
            taco_1 = micros();
            measure_ok = true;
            hall_period = (taco_1 - taco_0) / (2 * CONT_PULSE);
            motor_speed = 1000000 / (hall_period * 2);
            cont_hall_pulses = 0;
        }
        else
        {
            cont_hall_pulses++;
        }
    }
    oldHallA = newHallA;
    //-------------------------------------------------

    //-------------- ACTUADOR ------------------
    sensor = (hall_a << 2) + (hall_b << 1) + (hall_c);
    //Serial.println(sensor);

    if (sensor == CLOCKWISE[next_status])
    {
        switch (control_mode)
        {
        case FIX_MODE:
            digitalWrite(PIN_PWM_0, PWM_0[next_status]);
            digitalWrite(PIN_PWM_1, PWM_1[next_status]);
            digitalWrite(PIN_PWM_2, PWM_2[next_status]);
            digitalWrite(PIN_PWM_3, PWM_3[next_status]);
            digitalWrite(PIN_PWM_4, PWM_4[next_status]);
            digitalWrite(PIN_PWM_5, PWM_5[next_status]);
            break;

        case TOP_MODE:
        
            digitalWrite(PIN_PWM_0, PWM_0[next_status]);

            if (PWM_1[next_status])
            {
                PWM_Instance[1]->setPWM(PIN_PWM_1, DEFAULT_FREQ_PWM, DEFAULT_DUTY_CYCLE, true);
            }
            else
            {
                PWM_Instance[1]->setPWM(PIN_PWM_1, DEFAULT_FREQ_PWM, 0, true);
            }

            digitalWrite(PIN_PWM_2, PWM_2[next_status]);

            if (PWM_3[next_status])
            {
                PWM_Instance[3]->setPWM(PIN_PWM_3, DEFAULT_FREQ_PWM, DEFAULT_DUTY_CYCLE, true);
            }
            else
            {
                PWM_Instance[3]->setPWM(PIN_PWM_3, DEFAULT_FREQ_PWM, 0, true);
            }

            digitalWrite(PIN_PWM_4, PWM_4[next_status]);

            if (PWM_5[next_status])
            {
                PWM_Instance[5]->setPWM(PIN_PWM_5, DEFAULT_FREQ_PWM, DEFAULT_DUTY_CYCLE, true);
            }
            else
            {
                PWM_Instance[5]->setPWM(PIN_PWM_5, DEFAULT_FREQ_PWM, 0, true);
            }
            break;
        case BLOQ_MODE:
            digitalWrite(PIN_PWM_0, PWM_0[next_status]);
            if (PWM_1[next_status])
            {
                PWM_Instance[1]->setPWM(PIN_PWM_1, DEFAULT_FREQ_PWM, DUTY_BLOQ[bloq], true);
            }
            else
            {
                PWM_Instance[1]->setPWM(PIN_PWM_1, DEFAULT_FREQ_PWM, 0, true);
            }
            digitalWrite(PIN_PWM_2, PWM_2[next_status]);
            if (PWM_3[next_status])
            {
                PWM_Instance[3]->setPWM(PIN_PWM_3, DEFAULT_FREQ_PWM, DUTY_BLOQ[bloq], true);
            }
            else
            {
                PWM_Instance[3]->setPWM(PIN_PWM_3, DEFAULT_FREQ_PWM, 0, true);
            }
            digitalWrite(PIN_PWM_4, PWM_4[next_status]);
            if (PWM_5[next_status])
            {
                PWM_Instance[5]->setPWM(PIN_PWM_5, DEFAULT_FREQ_PWM, DUTY_BLOQ[bloq], true);
            }
            else
            {
                PWM_Instance[5]->setPWM(PIN_PWM_5, DEFAULT_FREQ_PWM, 0, true);
            }
            break;

        default:
            break;
        }

        next_status += 1;
        if (next_status > 5)
        {
            next_status = 0;
            // bloq++;
            // if (bloq > 3)
            // {
            //     bloq = 0;
            // }
        }
    }
}
void core1_entry()
{
    while (1)
    {
        /*          Buttons            */
        prev_button_frc_down = act_button_frc_down;
        prev_button_frc_up = act_button_frc_up;
        prev_button_on_off = act_button_on_off;
        prev_button_spd_down = act_button_spd_down;
        prev_button_spd_up = act_button_spd_up;
        act_button_frc_down = butt_frc_down.ReadState();
        act_button_on_off = butt_on_off.ReadState();
        act_button_frc_up = butt_frc_up.ReadState();
        act_button_spd_down = butt_spd_down.ReadState();
        act_button_spd_up = butt_spd_up.ReadState();
        prev_sample_button = sample_button;
        sample_button = debounce_button();

        if ((sample_button == LOW) && (prev_sample_button == HIGH))
        {
            bloq++;
            if (bloq > 3)
            {
                bloq = 0;
            }
        }

        /*          Refresh Display         */
        if (refresh_display.poll() != C_TIMER_NOT_EXPIRED)
        {
            display.fillRect(64, 0, 64, 128, BLACK);
            display.display();
            display.setCursor(70, 25);
            display.print(next_status);
            display.setCursor(70, 0);
            display.print(motor_speed);
            display.display();
            Serial.println("Esto es una prueba del Core Support");
            measure_ok = false;
            refresh_display.set(250);
        }
    }
}
bool debounce_button()
{
    int reading = digitalRead(PIN_EN_2);
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