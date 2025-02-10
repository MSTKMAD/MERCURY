/**
 * @file TestMotorFaulhaber.ino
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-10-30
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "Arduino.h"
#include "Adafruit_SSD1306.h"
#include "MilliTimer.h"
#include <Wire.h>
#include <pico/multicore.h>
#include "MCP4725.h"

const uint16_t LenDCDCLookupTable = 141; // 2V to 16V

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

const uint16_t C_PIN_TEST_3 = 0;
const uint16_t C_PIN_TEST_2 = 1;
const uint16_t C_PIN_R1 = 2;
const uint16_t C_PIN_R2 = 3;
const uint16_t C_PIN_SDA = 4;
const uint16_t C_PIN_SCL = 5;
const uint16_t C_PIN_R3 = 6;
const uint16_t C_PIN_R4 = 7;
const uint16_t C_PIN_POL_DUT = 9;
const uint16_t C_PIN_POL_LOAD = 8;
const uint16_t C_PIN_HALL = 10;
const uint16_t C_PIN_DUT = 11;
const uint16_t C_PIN_LED1 = 12;
const uint16_t C_PIN_LED2 = 13;
const uint16_t C_PIN_LED3 = 14;
const uint16_t C_PIN_LED4 = 15;
const uint16_t C_PIN_MISO = 16;
const uint16_t C_PIN_SS = 17;
const uint16_t C_PIN_SCK = 18;
const uint16_t C_PIN_MOSI = 19;
const uint16_t C_PIN_TEST_1 = 20;
const uint16_t C_PIN_EN_DCDC = 21;
const uint16_t C_PIN_DSP_RST = 22;
const uint16_t C_PIN_I_DUT = A0;
const uint16_t C_PIN_C_VSENSE_A = A1;
const uint16_t C_PIN_I_LOAD = A2;

const byte ADDR_I2C_DCDC = 0;
const int DCDC_ENABLED = HIGH;
const int DCDC_DISABLED = LOW;

const int C_NSAMPLES = 8;

const int C_CONV_IMOTOR = 0;
const int C_CONV_VSENSEA = 0;
const int C_CONV_VSENSE_2_SPEED = 0;

const int C_TIME_SPEED_MEAS_MS = 10000;

const int C_MODE_VOLT = 0x0B;
int mode = C_MODE_VOLT;

// FRAMES CONST
const int C_FRAME_0 = 0x0A;
const int C_FRAME_1 = 0x00;
const int C_FRAME_2 = 0x01;
const int C_FRAME_3 = 0x02;
const int C_FRAME_4 = 0x03;
const int C_FRAME_5 = 0x04;

int actual_frame;

Adafruit_SSD1306 display(128, 64, C_PIN_MOSI, C_PIN_SCK, C_PIN_MISO, C_PIN_DSP_RST, C_PIN_SS);
MCP4725 MCP(0x60, &Wire);

byte TPICvalue;
unsigned int DisplayValue;
uint16_t volts;
uint16_t en_dcdc;

uint16_t motor_load_current = 0;
uint16_t motor_load_voltage = 0;
uint16_t motor_dut_current = 0;
uint16_t motor_dut_voltage = 0;
bool newSense = false;

uint16_t prev_boton_1 = 0;
uint16_t actual_boton_1 = 0;
uint16_t prev_boton_2 = 0;
uint16_t actual_boton_2 = 0;
uint16_t prev_boton_3 = 0;
uint16_t actual_boton_3 = 0;

uint16_t res_on = 0;

bool pol_dut = false;  // Polarizad del Motor Dut F=Normal, T= Invertida
bool pol_load = false; // Polarizad del Motor Load F=Normal, T= Invertida

bool update_display = false;

const uint C_TOP_RESISTANCE = 48;
uint resistance = 0;

bool dut_control = false;     // Control del transsitor de enable de la alimentacion del motor DUT
bool en_dcdc_control = false; // Control del transsitor de enable del DCDC.

bool prev_hall_st = false;
bool actual_hall_st = false;

const int IMANES = 16;
int cont_pulses = 0;
uint32_t speed_dut = 0;
uint32_t t0 = 0;
uint32_t t1 = 0;
bool newSpeed = false;

// int escalera_resistencia = {inf, 270 , 220 , 180 , 121 , 108 , 99 , 72 , 47 , 40 , 38 , 37 , 33 , 32 , 31 , 28};
// int escalera_resistencia = {0  ,  1  ,  2  ,  4  ,  3  ,  5  ,  6 , 7  , 8  , 9  , 10 , 12 , 11 , 13 , 14 , 15};
int index_curva = 0;

const int TRAMOS = 10;
const int curva_0[20] = {50, 3, 50, 6, 50, 7, 50, 8, 50, 11, 50, 11, 50, 8, 50, 7, 50, 6, 50, 3};

const int C_TRAMO_1 = 0;
const int C_TRAMO_2 = 2;
const int C_TRAMO_3 = 4;
const int C_TRAMO_4 = 6;
const int C_TRAMO_5 = 8;
const int C_TRAMO_6 = 10;
const int C_TRAMO_7 = 12;
const int C_TRAMO_8 = 14;
const int C_TRAMO_9 = 16;
const int C_TRAMO_10 = 18;
MilliTimer timer_tramo_1, timer_tramo_2, timer_tramo_3, timer_tramo_4, timer_tramo_5, timer_tramo_6, timer_tramo_7, timer_tramo_8, timer_tramo_9, timer_tramo_10;
int tramo = C_TRAMO_1;

MilliTimer timer_sense;

// PWM

void setup()
{
    Serial.begin(115200);
    Wire.begin();
    analogReadResolution(12);

    pinMode(C_PIN_TEST_3, INPUT);
    pinMode(C_PIN_TEST_2, INPUT);
    pinMode(C_PIN_TEST_1, INPUT);
    pinMode(C_PIN_HALL, INPUT);
    pinMode(C_PIN_I_DUT, INPUT);
    pinMode(C_PIN_C_VSENSE_A, INPUT);
    pinMode(C_PIN_I_LOAD, INPUT);

    pinMode(C_PIN_R1, OUTPUT);
    pinMode(C_PIN_R2, OUTPUT);
    pinMode(C_PIN_R3, OUTPUT);
    pinMode(C_PIN_R4, OUTPUT);

    pinMode(C_PIN_LED1, OUTPUT);
    pinMode(C_PIN_LED2, OUTPUT);
    pinMode(C_PIN_LED3, OUTPUT);
    pinMode(C_PIN_LED4, OUTPUT);

    pinMode(C_PIN_POL_DUT, OUTPUT);
    pinMode(C_PIN_POL_LOAD, OUTPUT);
    pinMode(C_PIN_DUT, OUTPUT);
    pinMode(C_PIN_EN_DCDC, OUTPUT);

    digitalWrite(C_PIN_R2, LOW);
    digitalWrite(C_PIN_R1, LOW);
    digitalWrite(C_PIN_R3, LOW);
    digitalWrite(C_PIN_R4, LOW);

    digitalWrite(C_PIN_LED1, LOW);
    digitalWrite(C_PIN_LED2, LOW);
    digitalWrite(C_PIN_LED3, LOW);
    digitalWrite(C_PIN_LED4, LOW);

    digitalWrite(C_PIN_DUT, LOW);
    digitalWrite(C_PIN_EN_DCDC, HIGH);

    // Init State
    volts = 50; // Fijar voltaje
    en_dcdc = DCDC_ENABLED;
    digitalWrite(C_PIN_EN_DCDC, en_dcdc); // Activacion del DCDC
    pol_load = 1;
    pol_dut = 1;
    digitalWrite(C_PIN_POL_LOAD, pol_load);
}
// ------------------------------ LOOP ----------------------------------
void loop()
{
    SetVoltage(volts);
    digitalWrite(C_PIN_DUT, dut_control);
    SenseCurrent();
    UpdateLeds(dut_control, pol_dut);
}
void setup1()
{
    display.begin();
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.print("HOLA");
    display.display();
    actual_frame = C_FRAME_1;
    update_display = true;
    pol_dut = 1;
}
void loop1()
{
    // Serial.printf("%d-%.1f-%d\n", actual_frame, (float)volts / 10, update_display);
    LecturaBotones();
    if ((update_display == true) || (newSense == true))
    {
        newSense = false;
        update_display = false;
        display.clearDisplay();
        display.setTextSize(5);
        display.setTextColor(WHITE);
        display.setCursor(0, 5);
        display.print(int(volts / 10));

        display.setTextSize(2);
        display.setCursor(75, 0);

        display.print(motor_dut_current * 3300 / 4096 * 10 / 15);
        /*
        display.setCursor(64, 0);
        display.print((float)volts / 10);
        display.setCursor(0, 25);
        display.print("Sdut:");
        display.fillRect(64, 25, 100, 16, BLACK);
        display.setCursor(64, 25);
        display.print(speed_dut);
        */
        display.setCursor(0, 50);
        display.print("ON");
        display.setCursor(58, 50);
        display.print("+");
        display.setCursor(110, 50);
        display.print("-");
        display.display();
    }
    // BOTONES
    if ((actual_boton_1 == LOW) && (prev_boton_1 == HIGH))
    {
        if (dut_control == true)
        {
            dut_control = false;
        }
        else
        {
            dut_control = true;
        }
        update_display = true;
    }
    else if ((actual_boton_2 == LOW) && (prev_boton_2 == HIGH))
    {
        volts += 10;
        volts = constrain(volts, 20, 160);
        update_display = true;
    }
    else if ((actual_boton_3 == LOW) && (prev_boton_3 == HIGH))
    {
        volts -= 10;
        volts = constrain(volts, 20, 160);
        update_display = true;
    }
}
//------------------------------- FUNCIONES ---------------------------------
void SenseCurrent()
{
    if (timer_sense.poll(100) != C_TIMER_NOT_EXPIRED)
    {
        motor_load_voltage = 0;
        motor_dut_current = 0;
        motor_load_current = 0;
        for (int i = 0; i < C_NSAMPLES; i++)
        {
            motor_dut_current += analogRead(C_PIN_I_DUT);
            motor_load_current += analogRead(C_PIN_I_LOAD);
            motor_load_voltage += analogRead(C_PIN_C_VSENSE_A);
        }

        /*
            motor_load_current = motor_load_current * 3300 / 4096 * C_CONV_IMOTOR;  // valor en mA
            motor_load_voltage = motor_load_voltage * 3300 / 4096 * C_CONV_VSENSEA; // valor en mA
            motor_dut_speed = motor_load_voltage * C_CONV_VSENSE_2_SPEED;
        */

        motor_dut_current = motor_dut_current / C_NSAMPLES;
        motor_load_current = motor_load_current / C_NSAMPLES;
        motor_load_voltage = motor_load_voltage / C_NSAMPLES;
        newSense = true;
        
    }
}
void LecturaBotones()
{
    prev_boton_1 = actual_boton_1;
    prev_boton_2 = actual_boton_2;
    prev_boton_3 = actual_boton_3;
    actual_boton_1 = digitalRead(C_PIN_TEST_1);
    actual_boton_2 = digitalRead(C_PIN_TEST_2);
    actual_boton_3 = digitalRead(C_PIN_TEST_3);
}
void Write_TPIC2810(byte address, byte data)
{
    Wire.beginTransmission(byte(96)); // transmit command to device TPIC2810
    Wire.write(byte(68));             // Command to transfer next value to output register
    Wire.write(byte(data));
    Wire.endTransmission(); // stop transmitting
}
void SetVoltage(int v)
{
    v = constrain(v, 20, 160);                                // contrain del voltaje
    TPICvalue = pgm_read_byte_near(TPICLookupTable + v - 20); // Valor de I2C a enviar al TPIC
    Write_TPIC2810(ADDR_I2C_DCDC, TPICvalue);                 // Envio de la configuracion del TPIC
}

void UpdateLeds(bool dut_out, bool pol)
{
    digitalWrite(C_PIN_LED1, pol);
    digitalWrite(C_PIN_LED2, actual_hall_st);
    digitalWrite(C_PIN_LED3, LOW);
    digitalWrite(C_PIN_LED4, dut_out);
}