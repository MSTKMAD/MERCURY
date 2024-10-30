/**
 * @file ProtoSolenoide.ino
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-04-04
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <SPI.h>
#include <EEPROM.h>
#include <Wire.h>
#include <avr/pgmspace.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MilliTimer.h>
#include <pico/multicore.h>
#include <Adafruit_MCP4725.h>

#define OLED_DC 16
#define OLED_CS 17
#define OLED_CLK 18
#define OLED_MOSI 19
#define OLED_RESET 7

#define MAX_DUTY_MS 10
#define MAX_PHASE_MS 8

#define MAX_SPEED 200
#define MIN_SPEED 0

Adafruit_SSD1306 display(128, 64, OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

Adafruit_MCP4725 dac;

MilliTimer timer_pulling_Duty, timer_pulling_phase, timer_display_speed;
// === CONSTANTS === //
const uint16_t LenDCDCLookupTable = 141;                               // 2V to 16V
const static unsigned char __attribute__((progmem)) MusotokuLogo[] = { // NEW LOGO
    0x03, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00,
    0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80,
    0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0,
    0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60,
    0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
    0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18,
    0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c,
    0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04,
    0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04,
    0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04,
    0x80, 0x00, 0x41, 0x10, 0x83, 0xc1, 0xe0, 0x00, 0x04,
    0x80, 0x00, 0x63, 0x10, 0x84, 0x22, 0x10, 0x00, 0x04,
    0x80, 0x00, 0x77, 0x10, 0x88, 0x12, 0x10, 0x00, 0x04,
    0x80, 0x00, 0x5d, 0x10, 0x84, 0x02, 0x10, 0x00, 0x04,
    0x80, 0x00, 0x49, 0x10, 0x83, 0xc2, 0x10, 0x00, 0x04,
    0x80, 0x00, 0x41, 0x10, 0x80, 0x22, 0x10, 0x00, 0x04,
    0x80, 0x00, 0x41, 0x10, 0x80, 0x12, 0x10, 0x00, 0x04,
    0x80, 0x00, 0x41, 0x10, 0x88, 0x12, 0x10, 0x00, 0x04,
    0x80, 0x00, 0x41, 0x10, 0x84, 0x22, 0x10, 0x00, 0x04,
    0x80, 0x00, 0x41, 0x0f, 0x03, 0xc1, 0xa0, 0x00, 0x04,
    0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c,
    0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10,
    0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10,
    0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10,
    0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10,
    0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10,
    0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10,
    0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c,
    0x80, 0x00, 0x7f, 0x0f, 0x04, 0x22, 0x10, 0x00, 0x04,
    0x80, 0x00, 0x08, 0x10, 0x84, 0x42, 0x10, 0x00, 0x04,
    0x80, 0x00, 0x08, 0x10, 0x84, 0x82, 0x10, 0x00, 0x04,
    0x80, 0x00, 0x08, 0x10, 0x85, 0x02, 0x10, 0x00, 0x04,
    0x80, 0x00, 0x08, 0x10, 0x86, 0x02, 0x10, 0x00, 0x04,
    0x80, 0x00, 0x08, 0x10, 0x86, 0x02, 0x10, 0x00, 0x04,
    0x80, 0x00, 0x08, 0x10, 0x85, 0x02, 0x10, 0x00, 0x04,
    0x80, 0x00, 0x08, 0x10, 0x84, 0x82, 0x10, 0x00, 0x04,
    0x80, 0x00, 0x08, 0x10, 0x84, 0x42, 0x10, 0x00, 0x04,
    0x80, 0x00, 0x08, 0x0f, 0x04, 0x21, 0xe0, 0x00, 0x04,
    0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04,
    0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04,
    0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04,
    0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c,
    0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18,
    0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
    0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60,
    0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0,
    0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80,
    0x03, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00

};
PROGMEM const byte TPICLookupTable[LenDCDCLookupTable] = {
    200, 168, 104, 24, 152, 216, 56, 120, 4, 132, 196, 164, 228, 148, 84, 52, 180, 244, 140, 204, 172, 236, 28, 92, 220, 188, 124, 2,
    130, 194, 34, 98, 226, 146, 82, 50, 178, 242, 10, 74, 202, 170, 106, 26, 154, 218, 58, 122, 250, 134, 70, 38, 166, 230, 22, 86, 214,
    182, 118, 14, 142, 206, 46, 110, 238, 158, 94, 62, 190, 254, 129, 65, 193, 161, 97, 17, 145, 209, 49, 113, 241, 137, 73, 41, 169,
    233, 25, 89, 217, 185, 121, 5, 133, 197, 37, 101, 229, 149, 85, 53, 181, 245, 13, 77, 205, 173, 109, 29, 157, 221, 61, 125, 253,
    131, 67, 35, 163, 227, 19, 147, 83, 211, 51, 179, 115, 243, 139, 75, 203, 43, 171, 107, 235, 155, 91, 219, 123, 251, 7, 135, 199}; // OVERDRIVE version (+0.4v)
PROGMEM const byte DisplayValues[LenDCDCLookupTable] = {
    20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50,
    51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81,
    82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109,
    110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133,
    134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157,
    158, 159, 160};

const uint16_t C_DISPLAY_DUTY = 1;
const uint16_t C_DISPLAY_PH = 2;
const uint16_t C_DISPLAY_POWER = 3;
const uint16_t C_DISPLAY_SPEED = 4;

const uint16_t C_MODE_1 = 1;
const uint16_t C_MODE_2 = 2;
const uint16_t C_MODE_3 = 3;
const uint16_t C_MODE_4 = 4;

const int DEBOUNCE_ROTARY_TIME = 30;       //  milliseconds
const int DEBOUNCE_PUSHB_TIME = 100;       //  milliseconds
const unsigned long LONG_PRESS_TIME = 300; // Milliseconds

const int PUSHBUTTON_ON = LOW;
const int PUSHBUTTON_OFF = HIGH;

const int LOW_PITCH = 500;  // Microseconds between buzzer pulses
const int HIGH_PITCH = 200; // Microseconds between buzzer pulses

const byte BEEP_IS_TRUE = true;

const int MAX_DUTY = 100;
const int MIN_DUTY = 1;
/*  Pines   */

const uint16_t C_PIN_MOTOR_L = 6;
const uint16_t C_PIN_MOTOR_H = 21;
const uint16_t C_PIN_PH = A2;
const uint16_t C_PIN_DUTY = A0;
const uint16_t C_PIN_HALL_SENSOR = 9;

const int PUSHBUTTON_IDLE = 0;
const int PUSHBUTTON_ON_EDGE = 1;
const int PUSHBUTTON_FALL = 2;
const int PUSHBUTTON_LONGPRESS = 3;

const int ROTB = 12;
const int ROTA = 24;

const int PUSHBUTTON_IP = 3; // HIGH = OFF; LOW = ON

// === SIGNALS === //

unsigned long debouncerRotaryTime;
unsigned long Time; // Accounts for the General Time of the application (milliseconds from start up)
bool encoderA;
bool encoderALast = LOW;   // remembers the previous pin state
int encoderPos = 50;       // INDEX for the TPICLookupTable ==>  Vout = (encoderPos+20)-1  <----{ x10 range, i.e  110 = 11V }
int EncoderChange;         // Rotary spin right/left --> EncoderChange = +1/-1;
int EncoderChangePrev = 1; // Rotary spin right/left --> EncoderChange = +1/-1;
unsigned long StandbyGlobalTimer = 0;
byte RotaryDebounced = true;

int PushbuttonNow = PUSHBUTTON_OFF;
int PushbuttonAction = PUSHBUTTON_IDLE;
int PushbuttonPrev = PUSHBUTTON_OFF;
unsigned long debouncerPushTime;
unsigned long LongPressPushBTime;
int BlankingPushB_action = false; // After long press of pushbutton (enters into continuous mode). To avoid memory change also (fall edge) this variable signal the event

int PUSHB_Longpress_1st_Action = true; // If true then it is 1st action. THis avoids continuous entering in the LONGPRESS action

bool hallPrev;
bool hall;

bool trigger_hall = false;

uint16_t phase_us = 1;
uint16_t duty_us = 2;

byte TPICvalue;
unsigned int DisplayValue;

bool DisplayStatus = true;
uint32_t duty = 0;
uint32_t dutyPrev = 0;
uint32_t phase = 0;
uint32_t phasePrev = 0;

uint32_t dac_speed = 0;

uint16_t count_mode = 0;

uint32_t speed = 0;
uint32_t speedPrev = 0;

uint32_t time_0_hall = 0;

void setup()
{
    Wire.begin();
    dac.begin(0x60);
    display.begin(SSD1306_SWITCHCAPVCC);
    analogReadResolution(12);
    pinMode(PUSHBUTTON_IP, INPUT);
    pinMode(ROTA, INPUT);
    pinMode(ROTB, INPUT);
    pinMode(C_PIN_DUTY, INPUT);
    pinMode(C_PIN_PH, INPUT);
    pinMode(C_PIN_MOTOR_H, OUTPUT);
    pinMode(C_PIN_MOTOR_L, OUTPUT);

    debouncerPushTime = Time;

    display.clearDisplay();
    display.drawBitmap(29, 10, MusotokuLogo, 70, 48, WHITE);
    display.display();
    delay(500);
    display.clearDisplay();
    display.display();
    DisplayValue = pgm_read_byte_near(DisplayValues + encoderPos);
    TPICvalue = pgm_read_byte_near(TPICLookupTable + encoderPos);
    Wire.beginTransmission(byte(96)); // transmit command to device TPIC2810
    Wire.write(byte(68));             // Command to transfer next value to output register
    Wire.write(byte(TPICvalue));
    Wire.endTransmission(); // stop transmitting
    UpdateDisplay(C_DISPLAY_POWER);
    UpdateDisplay(C_DISPLAY_DUTY);
    UpdateDisplay(C_DISPLAY_PH);
    multicore_launch_core1(core1_entry);
    time_0_hall = micros();
    digitalWrite(C_PIN_MOTOR_L, LOW);
}

void loop()
{
    hallPrev = hall;
    hall = digitalRead(C_PIN_HALL_SENSOR);

    if ((hall == HIGH) && (hallPrev == LOW))
    {
        trigger_hall = true;
        speedPrev = speed;
        speed = 1000000 / (micros() - time_0_hall);
        speed = constrain(speed, MIN_SPEED, MAX_SPEED);
        time_0_hall = micros();
    }
    if (DisplayStatus)
    {
        if (trigger_hall == true)
        {
            delayMicroseconds(phase);

            digitalWrite(C_PIN_MOTOR_H, HIGH);

            delayMicroseconds(duty);

            digitalWrite(C_PIN_MOTOR_H, LOW);

            trigger_hall = false;
        }
    }
    else
    {

        digitalWrite(C_PIN_MOTOR_H, LOW);
    }
}

void core1_entry()
{
    while (1)
    {
        PushButtCheck();
        if (timer_display_speed.poll(100) != C_TIMER_NOT_EXPIRED)
        {
            if (speed != speedPrev)
            {
                UpdateDisplay(C_DISPLAY_SPEED);
            }
        }

        switch (count_mode)
        {
        case C_MODE_1:
            break;
        case C_MODE_2:
            break;
        case C_MODE_3:
            break;
        case C_MODE_4:
            break;

        default:
            RotaryCheck();
            DutyCheck();
            PhaseCheck();
            UpdateDisplaySpeed();
            break;
        }
    }
}
void RotaryCheck()
{
    // --------------------- HANDLE ROTARY ENCODER -----------------------------

    Time = millis();
    encoderA = digitalRead(ROTA);

    if ((encoderALast == HIGH) && (encoderA == LOW))
    {
        StandbyGlobalTimer = Time; // Reset the standby timer
        //--Detect Spin direction--
        if (digitalRead(ROTB) == LOW)
        {
            EncoderChange = -5;
        }
        else
        {
            EncoderChange = 5;
        }

        //--Debounding the spin--
        if (EncoderChange != EncoderChangePrev)
        {
            if ((Time - debouncerRotaryTime) >= DEBOUNCE_ROTARY_TIME)
            {
                RotaryDebounced = true; // Debounced --> Now change in spin direction is permited
            }
            else
            {
                RotaryDebounced = false;
            }
        }

        //--Performs Spin Action depending on Run Mode--
        if (RotaryDebounced == true)
        {
            EncoderChangePrev = EncoderChange; // EncoderChangePrev is ONLY updated once the spin direction change is consolidated
            debouncerRotaryTime = Time;        // Debouncing action counts from here!

            encoderPos = encoderPos + EncoderChange;
            encoderPos = constrain(encoderPos, 0, (LenDCDCLookupTable - 1));

            TPICvalue = pgm_read_byte_near(TPICLookupTable + encoderPos);

            Wire.beginTransmission(byte(96)); // transmit command to device TPIC2810
            Wire.write(byte(68));             // Command to transfer next value to output register
            Wire.write(byte(TPICvalue));
            Wire.endTransmission(); // stop transmitting

            DisplayValue = pgm_read_byte_near(DisplayValues + encoderPos);
            UpdateDisplay(C_DISPLAY_POWER);

        } // RotaryDebounce = true
    }
    encoderALast = encoderA;
}

void ReadPushbutton(int PIN_ip, int *StatNow, int *StatPrev, unsigned long *debouncerTimer, int DEBOUNCING_TIME, int *PushB_Event, byte BeepEnable, unsigned long *LongPressTimer)
{
    *StatNow = digitalRead(PIN_ip);
    if (*StatNow == PUSHBUTTON_ON)
    {
        if (*StatPrev == PUSHBUTTON_OFF)
        {
            *StatPrev = PUSHBUTTON_ON;

            if ((Time - *debouncerTimer) >= DEBOUNCING_TIME)
            {
                *PushB_Event = PUSHBUTTON_ON_EDGE;
                *LongPressTimer = Time;
            }
        }

        if (((Time - *LongPressTimer) >= LONG_PRESS_TIME)) // && (*PushB_Event != PUSHBUTTON_IDLE))
        {
            *PushB_Event = PUSHBUTTON_LONGPRESS;
        }
    }
    else //(PIN_IP = OFF)
    {
        if (*StatPrev == PUSHBUTTON_ON)
        {
            *StatPrev = PUSHBUTTON_OFF;
            *debouncerTimer = Time;

            if (*PushB_Event != PUSHBUTTON_IDLE)
            {
                *PushB_Event = PUSHBUTTON_FALL;
            }
        }
        else
        {
            *PushB_Event == PUSHBUTTON_IDLE;
        }
    }
}

void PushButtCheck()
{
    ReadPushbutton(PUSHBUTTON_IP, &PushbuttonNow, &PushbuttonPrev, &debouncerPushTime, DEBOUNCE_PUSHB_TIME, &PushbuttonAction, BEEP_IS_TRUE, &LongPressPushBTime);

    if ((PushbuttonAction == PUSHBUTTON_FALL)) // No  pushbutton available while longpress info is shown
    {
        if (BlankingPushB_action == false) // To avoid change memory after entering in continuous mode
        {
            PushbuttonAction = PUSHBUTTON_IDLE;
            if (DisplayStatus)
            {
                DisplayStatus = false;
                display.clearDisplay();
                display.display();
            }
            else
            {
                DisplayStatus = true;
                UpdateDisplay(C_DISPLAY_POWER);
                UpdateDisplay(C_DISPLAY_DUTY);
                UpdateDisplay(C_DISPLAY_PH);
            }
        }
        else
        {
            BlankingPushB_action = false;
        }
    }

    if (PushbuttonAction == PUSHBUTTON_LONGPRESS)
    {
        BlankingPushB_action = true; // This is to avoid change in memory value after the Longpress
        if (PUSHB_Longpress_1st_Action == true)
        {
            PUSHB_Longpress_1st_Action = false; // This avoids continuous entering in PushbuttonAction, which would result in toggling the continuous mode
            display.clearDisplay();             // clears the screen and buffer
            display.setTextSize(3);
            display.setTextColor(WHITE);
            display.drawRoundRect(0, 0, 128, 64, 10, WHITE);
            display.setCursor(33, 8);
            display.print("MODE");
            display.setCursor(43, 35);
            count_mode++;
            if (count_mode == 5)
            {
                count_mode = 1;
            }
            switch (count_mode)
            {
            case C_MODE_1:
                display.print("1");
                break;
            case C_MODE_2:
                display.print("2");
                break;
            case C_MODE_3:
                display.print("3");
                break;
            case C_MODE_4:
                display.print("4");
                break;

            default:
                break;
            }
            display.display();
            delay(500);
            display.clearDisplay();
            display.display();
        }
    }
    else
    {
        PUSHB_Longpress_1st_Action = true; // Reset the variable. Now is ready for the next cycle.
    }
}

void DutyCheck()
{
    if (timer_pulling_Duty.poll(100) != C_TIMER_NOT_EXPIRED)
    {
        dutyPrev = duty;
        duty = analogRead(C_PIN_DUTY);
        duty = 77 - (duty * 100 / 4095);
        duty = duty * MAX_DUTY_MS * 1000 / 77;
    }
    if (duty != dutyPrev)
    {
        UpdateDisplay(C_DISPLAY_DUTY);
    }
}

void PhaseCheck()
{
    if (timer_pulling_phase.poll(100) != C_TIMER_NOT_EXPIRED)
    {
        phasePrev = phase;
        phase = analogRead(C_PIN_PH);
        phase = 75 - (phase * 100 / 4095);
        phase = phase * MAX_PHASE_MS * 1000 / 75;
    }
    if (phase != phasePrev)
    {
        UpdateDisplay(C_DISPLAY_PH);
    }
}

void UpdateSolenoid()
{

    if (trigger_hall) // Generacion del pulso del solenoide
    {

        trigger_hall = false;
    }
}

void ShowDisplayValue(int value)
{
    int intPart;
    int fracPart;
    int cursor;

    // Config the text
    display.setTextSize(2);
    display.setTextColor(WHITE);

    display.fillRect(64, 0, 64, 32, BLACK); // clears the screen and buffer
    intPart = value / 10;
    fracPart = value - 10 * intPart;

    if (value >= 100)
    {
        // fracPart = value - 100;
        display.fillCircle(91, 13, 1, WHITE); // Decimal point 2
        cursor = 64;
    }
    else
    {
        // fracPart = value - 10*intPart;
        display.fillCircle(91, 13, 1, WHITE); // Decimal point 1
        cursor = 83;
    }
    display.setCursor(cursor, 0);
    display.print(intPart);
    display.setCursor(96, 0);
    display.println(fracPart);
}

void UpdateDisplay(uint16_t data_to_update)
{
    display.setTextSize(2);
    display.setTextColor(WHITE);

    switch (data_to_update)
    {
    case C_DISPLAY_SPEED:
        display.setCursor(0, 0);
        display.fillRect(0, 0, 64, 32, BLACK); // clears the screen and buffer
        display.print(speed);
        break;
    case C_DISPLAY_POWER:
        ShowDisplayValue(DisplayValue);
        break;
    case C_DISPLAY_PH:
        display.setCursor(0, 32);
        display.fillRect(0, 32, 64, 32, BLACK); // clears the screen and buffer
        display.print(phase);
        break;
    case C_DISPLAY_DUTY:
        display.setCursor(64, 32);
        display.fillRect(64, 32, 64, 32, BLACK); // clears the screen and buffer
        display.print(duty);
        break;

    default:
        break;
    }

    if (DisplayStatus)
    {
        display.display();
    }
    else
    {
        display.clearDisplay();
        display.display();
    }
}
void UpdateDisplaySpeed()
{

    dac_speed = speed * 4095 / MAX_SPEED;
    dac.setVoltage(dac_speed, false);
}