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

#define LOGIC_MODE 0
#define FIX_MODE 1
#define CONTROLED_MODE 2
#define BLOQ_MODE 3

//*******************************************************************//
//                           CONSTANTS                               //
//*******************************************************************//

/*      Pin     */
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
/*              */

const bool C_CLOCKWISE = true;
const bool C_COUNTERCLOCKWISE = false;

int CLOCKWISE[] = {5, 4, 6, 2, 3, 1};
const int COUNTER_CLOCKWISE[] = {1, 3, 2, 6, 4, 5};

const int HALL_STATE_0 = 5;
const int HALL_STATE_1 = 4;
const int HALL_STATE_2 = 6;
const int HALL_STATE_3 = 2;
const int HALL_STATE_4 = 3;
const int HALL_STATE_5 = 1;

const int NUM_OF_PWM = 12;
const int NUM_BLOQS = 4;

const int DEFAULT_FREQ_PWM = 15640;
const int DEFAULT_DUTY_CYCLE = 70;
const int DUTY_BLOQ[NUM_BLOQS] = {0, 25, 50, 75};

const int CONT_PULSE = 20;

const bool C_PRESSED = LOW;
const bool C_NOT_PRESSED = HIGH;

const int C_ADDRESS_I2C = 0x20;

const int MOTOR_RUNNING = 10;
const int MOTOR_STOP = 15;
const int MOTOR_LOOK = 20;

const int PULSOS_POR_VUELTA = 42;

const int INCREMENTO_MEDIDO = 360 / PULSOS_POR_VUELTA;

const int FREE_RUNING = false;

//*******************************************************************//
//                           Variables                               //
//*******************************************************************//

int bloq = 0;

int status;
int next_status = 0;

int newsensor;
int oldsensor;

bool measure_ok;

bool initOk = true;

bool hall_a;
bool hall_b;
bool hall_c;

int hall_period = 0;

bool oldHallA;
bool newHallA;
bool oldHallB;
bool newHallB;
bool oldHallC;
bool newHallC;

int cont_hall_pulses = 0;
uint32_t taco_0;
uint32_t taco_1;
uint32_t motor_speed;

int duty_cycle;
int controlled_duty_cycle = 50;

int control_mode = LOGIC_MODE;

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

bool valid_status = true;

bool breaking = false;
bool spinning_dir = C_CLOCKWISE;

bool screen = true;

bool new_data = false;
int motor_state = MOTOR_STOP;

int cont_pulsos = 0;

uint32_t angular_speed = 0;
uint32_t increment_time;

uint32_t t0, t1;
uint32_t t0A;
uint32_t t0B;
uint32_t t0C;

uint32_t tsensor_A;
uint32_t tsensor_B;
uint32_t tsensor_C;

int array_times_loop[36];

int array_times_loop_A[6];
int array_times_loop_B[6];
int array_times_loop_C[6];

//*******************************************************************//
//                      Functions & Classes                          //
//*******************************************************************//

RP2040_PWM *PWM_Instance[NUM_OF_PWM];
PCF8575 PCF(0x20);
Adafruit_SSD1306 display(128, 64, PIN_MOSI, PIN_SCK, PIN_MISO, PIN_RST, PIN_SS);
MilliTimer refresh_display;

void setup()
{
    Serial.begin(115200);
    PCF.begin();

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
    display.setCursor(0, 0);
    display.print("SPEED");
    display.setCursor(0, 25);
    display.print("DUTY");
    display.setCursor(0, 50);
    display.print("MODE");
    display.display();

    refresh_display.set(250);

    digitalWrite(PIN_EN_0, HIGH);
    digitalWrite(PIN_EN_1, HIGH);
    digitalWrite(PIN_EN_2, HIGH);
    digitalWrite(PIN_EN_3, HIGH);
    digitalWrite(PIN_EN_4, HIGH);
    digitalWrite(PIN_EN_5, HIGH);

    digitalWrite(PIN_PWM_0, LOW);
    digitalWrite(PIN_PWM_2, LOW);
    digitalWrite(PIN_PWM_4, LOW);

    multicore_launch_core1(core1_entry);
}

void loop()
{
    /*

    if (motor_state == MOTOR_RUNNING)
    {
        oldsensor = newsensor;
        oldHallA = newHallA;
        oldHallB = newHallB;
        oldHallC = newHallC;

        hall_a = digitalRead(PIN_HALL_A);
        hall_b = digitalRead(PIN_HALL_B);
        hall_c = digitalRead(PIN_HALL_C);

        newHallA = hall_a;
        newHallB = hall_b;
        newHallC = hall_c;

        newsensor = (hall_a << 2) + (hall_b << 1) + (hall_c);
        if (oldHallA != newHallA)
        {
            tsensor_A = micros() - t0A;
            t0A = micros();
        }
        if (oldHallB != newHallB)
        {
            tsensor_B = micros() - t0B;
            t0B = micros();
        }
        if (oldHallC != newHallC)
        {
            tsensor_C = micros() - t0C;
            t0C = micros();
        }
        if (oldsensor != newsensor)
        {
            cont_pulsos++;
            if (cont_pulsos == PULSOS_POR_VUELTA)
            {
                t0 = micros();

                cont_pulsos = 0;
                new_data = true;

                // motor_speed = 1000000 / angular_speed;
                // angular_speed = 0;
            }
            else
            {
                t1 = micros();
                increment_time = t1 - t0; // ms
                t0 = micros();
                // angular_speed += increment_time;
            }
            array_times_loop[cont_pulsos] = increment_time;

            if (!breaking)
            {
                switch (newsensor)
                {
                case HALL_STATE_0:
                    PWM_Instance[1]->setPWM(PIN_PWM_1, DEFAULT_FREQ_PWM, duty_cycle, true);
                    PWM_Instance[3]->setPWM(PIN_PWM_3, DEFAULT_FREQ_PWM, 0, true);
                    PWM_Instance[5]->setPWM(PIN_PWM_5, DEFAULT_FREQ_PWM, 0, true);
                    digitalWrite(PIN_PWM_0, LOW);
                    digitalWrite(PIN_PWM_2, HIGH);
                    digitalWrite(PIN_PWM_4, LOW);
                    break;
                case HALL_STATE_1:
                    PWM_Instance[1]->setPWM(PIN_PWM_1, DEFAULT_FREQ_PWM, duty_cycle, true);
                    PWM_Instance[3]->setPWM(PIN_PWM_3, DEFAULT_FREQ_PWM, 0, true);
                    PWM_Instance[5]->setPWM(PIN_PWM_5, DEFAULT_FREQ_PWM, 0, true);
                    digitalWrite(PIN_PWM_0, LOW);
                    digitalWrite(PIN_PWM_2, LOW);
                    digitalWrite(PIN_PWM_4, HIGH);
                    break;
                case HALL_STATE_2:
                    PWM_Instance[1]->setPWM(PIN_PWM_1, DEFAULT_FREQ_PWM, 0, true);
                    PWM_Instance[3]->setPWM(PIN_PWM_3, DEFAULT_FREQ_PWM, duty_cycle, true);
                    PWM_Instance[5]->setPWM(PIN_PWM_5, DEFAULT_FREQ_PWM, 0, true);
                    digitalWrite(PIN_PWM_0, LOW);
                    digitalWrite(PIN_PWM_2, LOW);
                    digitalWrite(PIN_PWM_4, HIGH);
                    break;
                case HALL_STATE_3:
                    PWM_Instance[1]->setPWM(PIN_PWM_1, DEFAULT_FREQ_PWM, 0, true);
                    PWM_Instance[3]->setPWM(PIN_PWM_3, DEFAULT_FREQ_PWM, duty_cycle, true);
                    PWM_Instance[5]->setPWM(PIN_PWM_5, DEFAULT_FREQ_PWM, 0, true);
                    digitalWrite(PIN_PWM_0, HIGH);
                    digitalWrite(PIN_PWM_2, LOW);
                    digitalWrite(PIN_PWM_4, LOW);
                    break;
                case HALL_STATE_4:
                    PWM_Instance[1]->setPWM(PIN_PWM_1, DEFAULT_FREQ_PWM, 0, true);
                    PWM_Instance[3]->setPWM(PIN_PWM_3, DEFAULT_FREQ_PWM, 0, true);
                    PWM_Instance[5]->setPWM(PIN_PWM_5, DEFAULT_FREQ_PWM, duty_cycle, true);
                    digitalWrite(PIN_PWM_0, HIGH);
                    digitalWrite(PIN_PWM_2, LOW);
                    digitalWrite(PIN_PWM_4, LOW);
                    break;
                case HALL_STATE_5:
                    PWM_Instance[1]->setPWM(PIN_PWM_1, DEFAULT_FREQ_PWM, 0, true);
                    PWM_Instance[3]->setPWM(PIN_PWM_3, DEFAULT_FREQ_PWM, 0, true);
                    PWM_Instance[5]->setPWM(PIN_PWM_5, DEFAULT_FREQ_PWM, duty_cycle, true);
                    digitalWrite(PIN_PWM_0, LOW);
                    digitalWrite(PIN_PWM_2, HIGH);
                    digitalWrite(PIN_PWM_4, LOW);
                    break;

                default:
                    break;
                }
            }
            else
            {
                PWM_Instance[1]->setPWM(PIN_PWM_1, DEFAULT_FREQ_PWM, 0, true);
                PWM_Instance[3]->setPWM(PIN_PWM_3, DEFAULT_FREQ_PWM, 0, true);
                PWM_Instance[5]->setPWM(PIN_PWM_5, DEFAULT_FREQ_PWM, 0, true);
                digitalWrite(PIN_PWM_0, HIGH);
                digitalWrite(PIN_PWM_2, HIGH);
                digitalWrite(PIN_PWM_4, HIGH);
            }
        }
    }
    else if (motor_state == MOTOR_STOP)
    {
         PWM_Instance[1]->setPWM(PIN_PWM_1, DEFAULT_FREQ_PWM, 0, true);
         PWM_Instance[3]->setPWM(PIN_PWM_3, DEFAULT_FREQ_PWM, 0, true);
         PWM_Instance[5]->setPWM(PIN_PWM_5, DEFAULT_FREQ_PWM, 0, true);
         digitalWrite(PIN_PWM_0, HIGH);
         digitalWrite(PIN_PWM_2, HIGH);
         digitalWrite(PIN_PWM_4, HIGH);

    }
    else if (motor_state == MOTOR_LOOK)
    {
        switch (newsensor)
        {
        case HALL_STATE_0:
            PWM_Instance[1]->setPWM(PIN_PWM_1, DEFAULT_FREQ_PWM, duty_cycle / 2, true);
            PWM_Instance[3]->setPWM(PIN_PWM_3, DEFAULT_FREQ_PWM, 0, true);
            PWM_Instance[5]->setPWM(PIN_PWM_5, DEFAULT_FREQ_PWM, 0, true);
            digitalWrite(PIN_PWM_0, LOW);
            digitalWrite(PIN_PWM_2, HIGH);
            digitalWrite(PIN_PWM_4, LOW);
            break;
        case HALL_STATE_1:
            PWM_Instance[1]->setPWM(PIN_PWM_1, DEFAULT_FREQ_PWM, duty_cycle / 2, true);
            PWM_Instance[3]->setPWM(PIN_PWM_3, DEFAULT_FREQ_PWM, 0, true);
            PWM_Instance[5]->setPWM(PIN_PWM_5, DEFAULT_FREQ_PWM, 0, true);
            digitalWrite(PIN_PWM_0, LOW);
            digitalWrite(PIN_PWM_2, LOW);
            digitalWrite(PIN_PWM_4, HIGH);
            break;
        case HALL_STATE_2:
            PWM_Instance[1]->setPWM(PIN_PWM_1, DEFAULT_FREQ_PWM, 0, true);
            PWM_Instance[3]->setPWM(PIN_PWM_3, DEFAULT_FREQ_PWM, duty_cycle / 2, true);
            PWM_Instance[5]->setPWM(PIN_PWM_5, DEFAULT_FREQ_PWM, 0, true);
            digitalWrite(PIN_PWM_0, LOW);
            digitalWrite(PIN_PWM_2, LOW);
            digitalWrite(PIN_PWM_4, HIGH);
            break;
        case HALL_STATE_3:
            PWM_Instance[1]->setPWM(PIN_PWM_1, DEFAULT_FREQ_PWM, 0, true);
            PWM_Instance[3]->setPWM(PIN_PWM_3, DEFAULT_FREQ_PWM, duty_cycle / 2, true);
            PWM_Instance[5]->setPWM(PIN_PWM_5, DEFAULT_FREQ_PWM, 0, true);
            digitalWrite(PIN_PWM_0, HIGH);
            digitalWrite(PIN_PWM_2, LOW);
            digitalWrite(PIN_PWM_4, LOW);
            break;
        case HALL_STATE_4:
            PWM_Instance[1]->setPWM(PIN_PWM_1, DEFAULT_FREQ_PWM, 0, true);
            PWM_Instance[3]->setPWM(PIN_PWM_3, DEFAULT_FREQ_PWM, 0, true);
            PWM_Instance[5]->setPWM(PIN_PWM_5, DEFAULT_FREQ_PWM, duty_cycle / 2, true);
            digitalWrite(PIN_PWM_0, HIGH);
            digitalWrite(PIN_PWM_2, LOW);
            digitalWrite(PIN_PWM_4, LOW);
            break;
        case HALL_STATE_5:
            PWM_Instance[1]->setPWM(PIN_PWM_1, DEFAULT_FREQ_PWM, 0, true);
            PWM_Instance[3]->setPWM(PIN_PWM_3, DEFAULT_FREQ_PWM, 0, true);
            PWM_Instance[5]->setPWM(PIN_PWM_5, DEFAULT_FREQ_PWM, duty_cycle / 2, true);
            digitalWrite(PIN_PWM_0, LOW);
            digitalWrite(PIN_PWM_2, HIGH);
            digitalWrite(PIN_PWM_4, LOW);
            break;

        default:
            break;
        }
    }
*/
}

void core1_entry()
{
    while (1)
    {
        switch (control_mode)
        {
        case LOGIC_MODE:
            duty_cycle = 100;
            break;
        case FIX_MODE:
            duty_cycle = DEFAULT_DUTY_CYCLE;
            break;
        case CONTROLED_MODE:
            duty_cycle = controlled_duty_cycle;
            break;
        case BLOQ_MODE:
            // duty_cycle = DUTY_BLOQ[bloq];
            break;
        default:
            break;
        }
        /*          Update Buttons            */
        prev_button_frc_down = act_button_frc_down;
        prev_button_frc_up = act_button_frc_up;
        prev_button_on_off = act_button_on_off;
        prev_button_spd_down = act_button_spd_down;
        prev_button_spd_up = act_button_spd_up;

        // act_button_frc_down = butt_frc_down.ReadState();
        // act_button_on_off = butt_on_off.ReadState();
        // act_button_frc_up = butt_frc_up.ReadState();
        // act_button_spd_down = butt_spd_down.ReadState();
        // act_button_spd_up = butt_spd_up.ReadState();

        act_button_frc_down = PCF.read(PIN_FRC_DOWN);
        act_button_on_off = PCF.read(PIN_ON_OFF);
        act_button_frc_up = PCF.read(PIN_FRC_UP);
        act_button_spd_down = PCF.read(PIN_SPD_DOWN);
        act_button_spd_up = PCF.read(PIN_SPD_UP);

        /*          Button Triggers         */

        //  ON/OFF
        if ((act_button_on_off == C_NOT_PRESSED) && (prev_button_on_off == C_PRESSED))
        {
            motor_state = MOTOR_STOP;
            delay(100);
            OneLoop();
            // motor_state = MOTOR_LOOK;
            Serial.printf("ONE LOOP \n");
            /* screen = !screen;
             if (screen)
             {
                 display.setTextColor(WHITE);
                 display.setCursor(0, 0);
                 display.print("SPEED");
                 display.setCursor(0, 25);
                 display.print("DUTY");
                 display.setCursor(0, 50);
                 display.print("MODE");
                 display.display();
             }*/
            /*
                        if (motor_state == MOTOR_STOP)
                        {
                            motor_state = MOTOR_RUNNING;
                        }
                        else if (motor_state == MOTOR_LOOK)
                        {
                            motor_state = MOTOR_RUNNING;
                        }
                        else if (motor_state == MOTOR_RUNNING)
                        {
                            motor_state = MOTOR_STOP;
                        }
            */
        }
        //  Force Up
        if ((act_button_frc_up == C_NOT_PRESSED) && (prev_button_frc_up == C_PRESSED))
        {
            control_mode++;
            control_mode = constrain(control_mode, LOGIC_MODE, BLOQ_MODE);
        }
        //  Force Down
        if ((act_button_frc_down == C_NOT_PRESSED) && (prev_button_frc_down == C_PRESSED))
        {
            control_mode--;
            control_mode = constrain(control_mode, LOGIC_MODE, BLOQ_MODE);
        }
        //  Speed Up
        if ((act_button_spd_up == C_NOT_PRESSED) && (prev_button_spd_up == C_PRESSED))
        {
            if (control_mode == CONTROLED_MODE)
            {
                controlled_duty_cycle += 10;
                controlled_duty_cycle = constrain(controlled_duty_cycle, 10, 90);
            }
        }
        //  Speed Down
        if ((act_button_spd_down == C_NOT_PRESSED) && (prev_button_spd_down == C_PRESSED))
        {
            if (control_mode == CONTROLED_MODE)
            {
                controlled_duty_cycle -= 10;
                controlled_duty_cycle = constrain(controlled_duty_cycle, 10, 90);
            }
        }

        /*          Refresh Display         */
        if (screen)
        {
            if (refresh_display.poll() != C_TIMER_NOT_EXPIRED)
            {
                display.fillRect(64, 0, 64, 64, BLACK);
                display.display();
                display.setCursor(70, 0);
                display.print(motor_speed);
                display.setCursor(70, 25);
                display.print(duty_cycle);
                switch (control_mode)
                {
                case LOGIC_MODE:
                    display.setCursor(70, 50);
                    display.print("LOGIC");
                    break;
                case FIX_MODE:
                    display.setCursor(70, 50);
                    display.print("FIX");
                    break;
                case CONTROLED_MODE:
                    display.setCursor(70, 50);
                    display.print("CONTOLLED");
                    break;
                case BLOQ_MODE:
                    display.setCursor(70, 50);
                    display.print("BLOCKs");
                    break;
                default:
                    break;
                }
                display.display();
                // Serial.println("Esto es una prueba del Core Support");
                measure_ok = false;
                refresh_display.set(250);
            }
        }
        else
        {
            display.fillRect(0, 0, 128, 64, BLACK);
            display.display();
        }

        if (new_data == true)
        {
            Serial.printf("/*");
            Serial.printf("%d,", tsensor_A);
            Serial.printf("%d,", tsensor_B);
            Serial.printf("%d,", tsensor_C);
            Serial.printf("*/\n");
            new_data = false;
        }
    }
}

void OpenTransistors()
{
    digitalWrite(PIN_PWM_1, LOW);
    digitalWrite(PIN_PWM_3, LOW);
    digitalWrite(PIN_PWM_5, LOW);
    digitalWrite(PIN_PWM_0, LOW);
    digitalWrite(PIN_PWM_2, LOW);
    digitalWrite(PIN_PWM_4, LOW);
}

void OneLoop()
{
    int loop_count = 0;
    int indx;
    int next_indx;
    int next_indx_2;
    int prev_indx;
    int prev_indx_2;
    int steps_error = 0;
    bool good = false;
    int objetive = PULSOS_POR_VUELTA;
    MilliTimer timer_idle;

    hall_a = digitalRead(PIN_HALL_A);
    hall_b = digitalRead(PIN_HALL_B);
    hall_c = digitalRead(PIN_HALL_C);
    newsensor = (hall_a << 2) + (hall_b << 1) + (hall_c);
    for (int i = 0; i < 6; i++)
    {
        if (newsensor == COUNTER_CLOCKWISE[i])
        {
            indx = i;
        }
    }
    duty_cycle = 20;
    while (!good)
    {
        while (loop_count < objetive) // CCW
        {
            oldsensor = newsensor;

            hall_a = digitalRead(PIN_HALL_A);
            hall_b = digitalRead(PIN_HALL_B);
            hall_c = digitalRead(PIN_HALL_C);

            newsensor = (hall_a << 2) + (hall_b << 1) + (hall_c);
            switch (newsensor) // CCW
            {
            case HALL_STATE_0:
                PWM_Instance[1]->setPWM(PIN_PWM_1, DEFAULT_FREQ_PWM, duty_cycle, true);
                PWM_Instance[3]->setPWM(PIN_PWM_3, DEFAULT_FREQ_PWM, 0, true);
                PWM_Instance[5]->setPWM(PIN_PWM_5, DEFAULT_FREQ_PWM, 0, true);
                digitalWrite(PIN_PWM_0, LOW);
                digitalWrite(PIN_PWM_2, HIGH);
                digitalWrite(PIN_PWM_4, LOW);
                break;
            case HALL_STATE_1:
                PWM_Instance[1]->setPWM(PIN_PWM_1, DEFAULT_FREQ_PWM, duty_cycle, true);
                PWM_Instance[3]->setPWM(PIN_PWM_3, DEFAULT_FREQ_PWM, 0, true);
                PWM_Instance[5]->setPWM(PIN_PWM_5, DEFAULT_FREQ_PWM, 0, true);
                digitalWrite(PIN_PWM_0, LOW);
                digitalWrite(PIN_PWM_2, LOW);
                digitalWrite(PIN_PWM_4, HIGH);
                break;
            case HALL_STATE_2:
                PWM_Instance[1]->setPWM(PIN_PWM_1, DEFAULT_FREQ_PWM, 0, true);
                PWM_Instance[3]->setPWM(PIN_PWM_3, DEFAULT_FREQ_PWM, duty_cycle, true);
                PWM_Instance[5]->setPWM(PIN_PWM_5, DEFAULT_FREQ_PWM, 0, true);
                digitalWrite(PIN_PWM_0, LOW);
                digitalWrite(PIN_PWM_2, LOW);
                digitalWrite(PIN_PWM_4, HIGH);
                break;
            case HALL_STATE_3:
                PWM_Instance[1]->setPWM(PIN_PWM_1, DEFAULT_FREQ_PWM, 0, true);
                PWM_Instance[3]->setPWM(PIN_PWM_3, DEFAULT_FREQ_PWM, duty_cycle, true);
                PWM_Instance[5]->setPWM(PIN_PWM_5, DEFAULT_FREQ_PWM, 0, true);
                digitalWrite(PIN_PWM_0, HIGH);
                digitalWrite(PIN_PWM_2, LOW);
                digitalWrite(PIN_PWM_4, LOW);
                break;
            case HALL_STATE_4:
                PWM_Instance[1]->setPWM(PIN_PWM_1, DEFAULT_FREQ_PWM, 0, true);
                PWM_Instance[3]->setPWM(PIN_PWM_3, DEFAULT_FREQ_PWM, 0, true);
                PWM_Instance[5]->setPWM(PIN_PWM_5, DEFAULT_FREQ_PWM, duty_cycle, true);
                digitalWrite(PIN_PWM_0, HIGH);
                digitalWrite(PIN_PWM_2, LOW);
                digitalWrite(PIN_PWM_4, LOW);
                break;
            case HALL_STATE_5:
                PWM_Instance[1]->setPWM(PIN_PWM_1, DEFAULT_FREQ_PWM, 0, true);
                PWM_Instance[3]->setPWM(PIN_PWM_3, DEFAULT_FREQ_PWM, 0, true);
                PWM_Instance[5]->setPWM(PIN_PWM_5, DEFAULT_FREQ_PWM, duty_cycle, true);
                digitalWrite(PIN_PWM_0, LOW);
                digitalWrite(PIN_PWM_2, HIGH);
                digitalWrite(PIN_PWM_4, LOW);
                break;

            default:
                break;
            }

            next_indx = indx + 1 >= 6 ? 0 : indx + 1;
            next_indx_2 = indx + 2 < 6 ? indx + 2 : (indx + 2 == 7 ? 1 : 0);
            if (newsensor == COUNTER_CLOCKWISE[next_indx])
            {
                indx = next_indx;
                loop_count++;
                Serial.println(loop_count);
            }
            else if (newsensor == COUNTER_CLOCKWISE[next_indx_2])
            {
                indx = next_indx_2;
                loop_count += 2;
                Serial.println(loop_count);
            }
            // Serial.printf("%d-%d-%d\n", hall_a, hall_b, hall_c);
            /*
            if (oldsensor != newsensor)
            {
                Serial.printf("%d-%d-%d\n",hall_a,hall_b,hall_c);

            }*/
        }
        timer_idle.set(100);
        while (timer_idle.poll() == C_TIMER_NOT_EXPIRED)
        {
            oldsensor = newsensor;
            hall_a = digitalRead(PIN_HALL_A);
            hall_b = digitalRead(PIN_HALL_B);
            hall_c = digitalRead(PIN_HALL_C);
            newsensor = (hall_a << 2) + (hall_b << 1) + (hall_c);

            next_indx = indx + 1 >= 6 ? 0 : indx + 1;
            next_indx_2 = indx + 2 < 6 ? indx + 2 : (indx + 2 == 7 ? 1 : 0);
            prev_indx = indx - 1 < 0 ? 5 : indx - 1;
            prev_indx_2 = indx - 2 >= 0 ? indx - 2 : (indx - 2 == -1 ? 5 : 4);
            if (newsensor == COUNTER_CLOCKWISE[next_indx])
            {
                Serial.printf("%d-%d-%d-%d-%d\n", prev_indx_2, prev_indx, indx, next_indx, next_indx_2);
                indx = next_indx;
                loop_count++;
                Serial.println(loop_count);
            }
            else if (newsensor == COUNTER_CLOCKWISE[next_indx_2])
            {
                Serial.printf("%d-%d-%d-%d-%d\n", prev_indx_2, prev_indx, indx, next_indx, next_indx_2);
                indx = next_indx_2;
                loop_count += 2;
                Serial.println(loop_count);
            }
            else if (newsensor == COUNTER_CLOCKWISE[prev_indx])
            {
                Serial.printf("%d-%d-%d-%d-%d\n", prev_indx_2, prev_indx, indx, next_indx, next_indx_2);
                indx = prev_indx;
                loop_count--;
                Serial.println(loop_count);
            }
            else if (newsensor == COUNTER_CLOCKWISE[prev_indx_2])
            {
                Serial.printf("%d-%d-%d-%d-%d\n", prev_indx_2, prev_indx, indx, next_indx, next_indx_2);
                indx = prev_indx_2;
                loop_count -= 2;
                Serial.println(loop_count);
            }
        }

        while (loop_count >= objetive)
        {
            hall_a = digitalRead(PIN_HALL_A);
            hall_b = digitalRead(PIN_HALL_B);
            hall_c = digitalRead(PIN_HALL_C);

            newHallA = hall_a;
            newHallB = hall_b;
            newHallC = hall_c;

            newsensor = (hall_a << 2) + (hall_b << 1) + (hall_c);
            switch (newsensor) // CW
            {
            case HALL_STATE_0:
                PWM_Instance[1]->setPWM(PIN_PWM_1, DEFAULT_FREQ_PWM, 0, true);
                PWM_Instance[3]->setPWM(PIN_PWM_3, DEFAULT_FREQ_PWM, duty_cycle, true);
                PWM_Instance[5]->setPWM(PIN_PWM_5, DEFAULT_FREQ_PWM, 0, true);
                digitalWrite(PIN_PWM_0, HIGH);
                digitalWrite(PIN_PWM_2, LOW);
                digitalWrite(PIN_PWM_4, LOW);
                break;
            case HALL_STATE_1:
                PWM_Instance[1]->setPWM(PIN_PWM_1, DEFAULT_FREQ_PWM, 0, true);
                PWM_Instance[3]->setPWM(PIN_PWM_3, DEFAULT_FREQ_PWM, 0, true);
                PWM_Instance[5]->setPWM(PIN_PWM_5, DEFAULT_FREQ_PWM, duty_cycle, true);
                digitalWrite(PIN_PWM_0, HIGH);
                digitalWrite(PIN_PWM_2, LOW);
                digitalWrite(PIN_PWM_4, LOW);
                break;
            case HALL_STATE_2:
                PWM_Instance[1]->setPWM(PIN_PWM_1, DEFAULT_FREQ_PWM, 0, true);
                PWM_Instance[3]->setPWM(PIN_PWM_3, DEFAULT_FREQ_PWM, 0, true);
                PWM_Instance[5]->setPWM(PIN_PWM_5, DEFAULT_FREQ_PWM, duty_cycle, true);
                digitalWrite(PIN_PWM_0, LOW);
                digitalWrite(PIN_PWM_2, HIGH);
                digitalWrite(PIN_PWM_4, LOW);
                break;
            case HALL_STATE_3:
                PWM_Instance[1]->setPWM(PIN_PWM_1, DEFAULT_FREQ_PWM, duty_cycle, true);
                PWM_Instance[3]->setPWM(PIN_PWM_3, DEFAULT_FREQ_PWM, 0, true);
                PWM_Instance[5]->setPWM(PIN_PWM_5, DEFAULT_FREQ_PWM, 0, true);
                digitalWrite(PIN_PWM_0, LOW);
                digitalWrite(PIN_PWM_2, HIGH);
                digitalWrite(PIN_PWM_4, LOW);
                break;
            case HALL_STATE_4:
                PWM_Instance[1]->setPWM(PIN_PWM_1, DEFAULT_FREQ_PWM, duty_cycle, true);
                PWM_Instance[3]->setPWM(PIN_PWM_3, DEFAULT_FREQ_PWM, 0, true);
                PWM_Instance[5]->setPWM(PIN_PWM_5, DEFAULT_FREQ_PWM, 0, true);
                digitalWrite(PIN_PWM_0, LOW);
                digitalWrite(PIN_PWM_2, LOW);
                digitalWrite(PIN_PWM_4, HIGH);
                break;
            case HALL_STATE_5:
                PWM_Instance[1]->setPWM(PIN_PWM_1, DEFAULT_FREQ_PWM, 0, true);
                PWM_Instance[3]->setPWM(PIN_PWM_3, DEFAULT_FREQ_PWM, duty_cycle, true);
                PWM_Instance[5]->setPWM(PIN_PWM_5, DEFAULT_FREQ_PWM, 0, true);
                digitalWrite(PIN_PWM_0, LOW);
                digitalWrite(PIN_PWM_2, LOW);
                digitalWrite(PIN_PWM_4, HIGH);
                break;

            default:
                break;
            }
            next_indx = indx + 1 >= 6 ? 0 : indx + 1;
            next_indx_2 = indx + 2 < 6 ? indx + 2 : (indx + 2 == 7 ? 1 : 0);
            if (newsensor == CLOCKWISE[next_indx])
            {
                indx = next_indx;
                loop_count--;
                Serial.println(loop_count);
            }
            else if (newsensor == CLOCKWISE[next_indx_2])
            {
                indx = next_indx_2;
                loop_count -= 2;
                Serial.println(loop_count);
            }
        }

        timer_idle.set(100);
        while (timer_idle.poll() == C_TIMER_NOT_EXPIRED)
        {
            oldsensor = newsensor;
            oldHallA = newHallA;
            oldHallB = newHallB;
            oldHallC = newHallC;

            hall_a = digitalRead(PIN_HALL_A);
            hall_b = digitalRead(PIN_HALL_B);
            hall_c = digitalRead(PIN_HALL_C);

            newHallA = hall_a;
            newHallB = hall_b;
            newHallC = hall_c;

            newsensor = (hall_a << 2) + (hall_b << 1) + (hall_c);

            next_indx = indx + 1 >= 6 ? 0 : indx + 1;
            next_indx_2 = indx + 2 < 6 ? indx + 2 : (indx + 2 == 7 ? 1 : 0);
            prev_indx = indx - 1 < 0 ? 5 : indx - 1;
            prev_indx_2 = indx - 2 >= 0 ? indx - 2 : (indx - 2 == -1 ? 5 : 4);
            if (newsensor == COUNTER_CLOCKWISE[next_indx])
            {
                Serial.printf("%d-%d-%d-%d-%d\n", prev_indx_2, prev_indx, indx, next_indx, next_indx_2);
                indx = next_indx;
                loop_count--;
                Serial.println(loop_count);
            }
            else if (newsensor == COUNTER_CLOCKWISE[next_indx_2])
            {
                Serial.printf("%d-%d-%d-%d-%d\n", prev_indx_2, prev_indx, indx, next_indx, next_indx_2);
                indx = next_indx_2;
                loop_count -= 2;
                Serial.println(loop_count);
            }
            else if (newsensor == COUNTER_CLOCKWISE[prev_indx])
            {
                Serial.printf("%d-%d-%d-%d-%d\n", prev_indx_2, prev_indx, indx, next_indx, next_indx_2);
                indx = prev_indx;
                loop_count++;
                Serial.println(loop_count);
            }
            else if (newsensor == COUNTER_CLOCKWISE[prev_indx_2])
            {
                Serial.printf("%d-%d-%d-%d-%d\n", prev_indx_2, prev_indx, indx, next_indx, next_indx_2);
                indx = prev_indx_2;
                loop_count += 2;
                Serial.println(loop_count);
            }
        }
        if (loop_count == objetive)
        {
            good = true;
        }else{
            duty_cycle--;
            duty_cycle = constrain(duty_cycle, 10, 20);
        }
    }
}