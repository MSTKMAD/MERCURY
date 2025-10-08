/**
 * @file TestMachines.ino
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2025-05-16
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "Arduino.h"
#include "Adafruit_SSD1306.h"
#include "MilliTimer.h"
#include <Wire.h>
#include <pico/multicore.h>
#include <MCP23017.h>
#include "SD.h"
#include "SPI.h"
#include "EEPROM.h"

#define MCP23017_ADDR 0x20
#define VERSION "1.0.0"
#define T_SAMPLE "1 second"

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

const uint16_t C_PIN_TEST_1 = 6;
const uint16_t C_PIN_TEST_2 = 20;
const uint16_t C_PIN_TEST_3 = 1;
const uint16_t C_PIN_TEST_4 = 0;

const uint16_t C_PIN_SDA = 4;
const uint16_t C_PIN_SCL = 5;
const uint16_t C_PIN_POL_DUT = 9;
const uint16_t C_PIN_SW_GND = 11;

const uint16_t C_PIN_LED1 = 14;
const uint16_t C_PIN_LED2 = 13;
const uint16_t C_PIN_LED3 = 12;

const uint16_t C_PIN_SS_SD = 15;
const uint16_t C_PIN_DC = 10;
const uint16_t C_PIN_SS_LED = 17;
const uint16_t C_PIN_SCK = 18;
const uint16_t C_PIN_MOSI = 19;
const uint16_t C_PIN_EN_DCDC = 21;
const uint16_t C_PIN_DSP_RST = 22;
const uint16_t C_PIN_I_FREC = A1;
const uint16_t C_PIN_I_MOTOR = A2;

const uint16_t C_PIN_SW_M1 = 0;
const uint16_t C_PIN_SW_M2 = 2;
const uint16_t C_PIN_SW_M3 = 4;
const uint16_t C_PIN_SW_M4 = 6;
const uint16_t C_PIN_SW_M5 = 8;
const uint16_t C_PIN_SW_M6 = 10;
const uint16_t C_PIN_SW_M7 = 12;
const uint16_t C_PIN_SW_M8 = 14;
const uint16_t C_PIN_SW_M9 = 3;
const uint16_t C_PIN_SW_M10 = 1;
const uint16_t C_PIN_SW_M11 = 3;
const uint16_t C_PIN_SW_M12 = 5;
const uint16_t C_PIN_SW_M13 = 7;
const uint16_t C_PIN_SW_M14 = 9;
const uint16_t C_PIN_SW_M15 = 11;
const uint16_t C_PIN_SW_M16 = 13;
const uint16_t C_PIN_SW_M17 = 15;
const uint16_t C_PIN_SW_M18 = 2;

const bool C_RELE_VOUT = 0;
const bool C_RELE_GND = 1;

uint16_t reles[18] = {C_PIN_SW_M1, C_PIN_SW_M2, C_PIN_SW_M3, C_PIN_SW_M4, C_PIN_SW_M5,
                      C_PIN_SW_M6, C_PIN_SW_M7, C_PIN_SW_M8, C_PIN_SW_M9, C_PIN_SW_M10,
                      C_PIN_SW_M11, C_PIN_SW_M12, C_PIN_SW_M13, C_PIN_SW_M14, C_PIN_SW_M15,
                      C_PIN_SW_M16, C_PIN_SW_M17, C_PIN_SW_M18};
uint16_t reles_index_samples[18] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

const uint16_t C_NSAMPLES = 8; // Cantidad de muestras para el promedio

const byte ADDR_I2C_DCDC = 0;
const int DCDC_ENABLED = HIGH;
const int DCDC_DISABLED = LOW;

const int EEPROM_UID_ADDR = 0x00; // Direccion de la EEPROM para el UID

Adafruit_SSD1306 display(128, 64, &SPI, C_PIN_DC, C_PIN_DSP_RST, C_PIN_SS_LED);
MilliTimer timer_sense, timerSeconds, timer_swap_reles, timer_ciclo_testeo, timer_sense_SD;

MCP23017 mcp = MCP23017(MCP23017_ADDR);
byte TPICvalue;
unsigned int DisplayValue;
uint16_t volts;
uint16_t en_dcdc;

uint16_t motor_frec_current = 0;
uint16_t motor_current = 0;
bool newSense = false;
bool newSDRead = false;

uint16_t prev_boton_1 = 0;
uint16_t actual_boton_1 = 0;
uint16_t prev_boton_2 = 0;
uint16_t actual_boton_2 = 0;
uint16_t prev_boton_3 = 0;
uint16_t actual_boton_3 = 0;
uint16_t prev_boton_4 = 0;
uint16_t actual_boton_4 = 0;

bool click_btn_1, click_btn_2, click_btn_3, click_btn_4;

bool led_1 = true, led_2 = true, led_3 = true;

uint16_t reles_states[18];

uint16_t cont_rele = 0;
uint16_t cont_secs = 0;
uint16_t cont_min = 0;
uint16_t cont_hours = 0;

const uint16_t C_MCHN_ST_HOLD = 0;
const uint16_t C_MCHN_ST_TESTING = 1;
const uint16_t C_MCHN_ST_ERROR = 2;
const uint16_t C_MCHN_ST_STOP = 3;

uint16_t state_machine_status = C_MCHN_ST_HOLD;

bool switch_gnd = false;
bool is15SecondsActive = false;

bool SPI_Enabled = false; // Variable para controlar el estado del SPI

uint16_t rele_actual = 17;
uint16_t rele_anterior = 0;

uint16_t uid = 0; // Variable para el UID de la EEPROM

int t0 = 0;
String Filenames[18] = {"machine_1.csv", "machine_2.csv", "machine_3.csv", "machine_4.csv",
                        "machine_5.csv", "machine_6.csv", "machine_7.csv", "machine_8.csv",
                        "machine_9.csv", "machine_10.csv", "machine_11.csv", "machine_12.csv",
                        "machine_13.csv", "machine_14.csv", "machine_15.csv", "machine_16.csv",
                        "machine_17.csv", "machine_18.csv"};
void setup()
{
    Wire.begin();
    Serial.begin(115200);
    EEPROM.begin(512); // Inicializa la EEPROM con 512 bytes
    //delay(1000);       // Espera para estabilizar la comunicacion serial
    Serial.print("Initializing...");
    mcp.init();
    mcp.portMode(MCP23017Port::A, 0); // Port A as output
    mcp.portMode(MCP23017Port::B, 0); // Port B as output
    analogReadResolution(12);

    pinMode(C_PIN_TEST_1, INPUT);
    pinMode(C_PIN_TEST_2, INPUT);
    pinMode(C_PIN_TEST_3, INPUT);
    pinMode(C_PIN_TEST_4, INPUT);

    pinMode(C_PIN_SW_M18, OUTPUT);
    pinMode(C_PIN_SW_M9, OUTPUT);
    pinMode(C_PIN_SW_GND, OUTPUT);

    pinMode(C_PIN_I_FREC, INPUT);
    pinMode(C_PIN_I_MOTOR, INPUT);

    pinMode(C_PIN_LED1, OUTPUT);
    pinMode(C_PIN_LED2, OUTPUT);
    pinMode(C_PIN_LED3, OUTPUT);

    pinMode(C_PIN_POL_DUT, OUTPUT);
    pinMode(C_PIN_EN_DCDC, OUTPUT);

    // Init State

    volts = 70; // Fijar voltaje
    en_dcdc = DCDC_ENABLED;
    digitalWrite(C_PIN_EN_DCDC, en_dcdc); // Activacion del DCDC
    SetVoltage(volts);

    Serial.println("Setup done");
    digitalWrite(C_PIN_SW_GND, HIGH); // Desactiva el switch de GND

    cont_rele = 0;                  // Reinicia el contador de reles
    timerSeconds.set(1000);         // 1 segundo
    reles_states[17] = C_RELE_VOUT; // Activa el rele 18
    reles_states[0] = C_RELE_GND;   // Activa el rele 1
    digitalWrite(C_PIN_SW_M18, C_RELE_VOUT);
    mcp.digitalWrite(reles[0], C_RELE_GND); // Activa el rele 1
}
// ------------------------------ LOOP ----------------------------------
void loop()
{

    UpdateLeds();
    LecturaBotones();
    SetVoltage(volts);
    SenseCurrent();

    switch (state_machine_status)
    {
    case C_MCHN_ST_HOLD:
        led_1 = false;
        led_2 = false;
        led_3 = true;
        if (click_btn_1)
        {
            state_machine_status = C_MCHN_ST_TESTING;
        }
        else if (click_btn_2)
        {
            state_machine_status = C_MCHN_ST_ERROR;
        }
        else if (click_btn_3)
        {
            state_machine_status = C_MCHN_ST_STOP;
        }
        else if (click_btn_4)
        {
            state_machine_status = C_MCHN_ST_HOLD;
        }

        break;

    case C_MCHN_ST_TESTING:
        led_1 = true;
        led_2 = false;
        led_3 = false;

        if (timerSeconds.poll() != C_TIMER_NOT_EXPIRED)
        {
            timerSeconds.set(1000); // Reinicia el temporizador a 1 segundo

            cont_secs++;

            Serial.print(cont_secs);
            if (cont_secs % 30 == 0)
            {
                cont_rele = cont_rele + 1;
                if (cont_rele >= 18)
                {
                    cont_rele = 0; // Reinicia el contador de reles
                }
                Serial.println("-----------------");
                Serial.printf("Rele actual: %d\n", cont_rele + 1);
                switch (cont_rele)
                {
                case 0:
                    reles_states[17] = C_RELE_VOUT; // Activa el rele 18
                    reles_states[0] = C_RELE_GND;   // Activa el rele 1
                    digitalWrite(C_PIN_SW_M18, C_RELE_VOUT);
                    mcp.digitalWrite(reles[0], C_RELE_GND); // Activa el rele 1
                    break;

                case 1:
                    reles_states[0] = C_RELE_VOUT;           // Activa el rele 1
                    reles_states[1] = C_RELE_GND;            // Activa el rele 2
                    mcp.digitalWrite(reles[0], C_RELE_VOUT); // Activa el rele 1
                    mcp.digitalWrite(reles[1], C_RELE_GND);  // Activa el rele 2
                    break;
                case 2:
                    reles_states[1] = C_RELE_VOUT;           // Activa el rele 2
                    reles_states[2] = C_RELE_GND;            // Activa el rele 3
                    mcp.digitalWrite(reles[1], C_RELE_VOUT); // Activa el rele 2
                    mcp.digitalWrite(reles[2], C_RELE_GND);  // Activa el rele 3
                    break;
                case 3:
                    reles_states[2] = C_RELE_VOUT;           // Activa el rele 3
                    reles_states[3] = C_RELE_GND;            // Activa el rele 4
                    mcp.digitalWrite(reles[2], C_RELE_VOUT); // Activa el rele 3
                    mcp.digitalWrite(reles[3], C_RELE_GND);  // Activa el rele 4
                    break;
                case 4:
                    reles_states[3] = C_RELE_VOUT;           // Activa el rele 4
                    reles_states[4] = C_RELE_GND;            // Activa el rele 5
                    mcp.digitalWrite(reles[3], C_RELE_VOUT); // Activa el rele 4
                    mcp.digitalWrite(reles[4], C_RELE_GND);  // Activa el rele 5
                    break;
                case 5:
                    reles_states[4] = C_RELE_VOUT;           // Activa el rele 5
                    reles_states[5] = C_RELE_GND;            // Activa el rele 6
                    mcp.digitalWrite(reles[4], C_RELE_VOUT); // Activa el rele 5
                    mcp.digitalWrite(reles[5], C_RELE_GND);  // Activa el rele 6
                    break;
                case 6:
                    reles_states[5] = C_RELE_VOUT;           // Activa el rele 4
                    reles_states[6] = C_RELE_GND;            // Activa el rele 5
                    mcp.digitalWrite(reles[5], C_RELE_VOUT); // Activa el rele 6
                    mcp.digitalWrite(reles[6], C_RELE_GND);  // Activa el rele 7
                    break;
                case 7:
                    reles_states[6] = C_RELE_VOUT;           // Activa el rele 4
                    reles_states[7] = C_RELE_GND;            // Activa el rele 5
                    mcp.digitalWrite(reles[6], C_RELE_VOUT); // Activa el rele 7
                    mcp.digitalWrite(reles[7], C_RELE_GND);  // Activa el rele 8
                    break;
                case 8:
                    reles_states[7] = C_RELE_VOUT;           // Activa el rele 4
                    reles_states[8] = C_RELE_GND;            // Activa el rele 5
                    mcp.digitalWrite(reles[7], C_RELE_VOUT); // Activa el rele 8
                    digitalWrite(C_PIN_SW_M9, C_RELE_GND);   // Activa el rele 9
                    break;
                case 9:
                    reles_states[8] = C_RELE_VOUT;          // Activa el rele 4
                    reles_states[9] = C_RELE_GND;           // Activa el rele 5
                    digitalWrite(C_PIN_SW_M9, C_RELE_VOUT); // Activa el rele 18
                    mcp.digitalWrite(reles[9], C_RELE_GND); // Activa el rele 10
                    break;
                case 10:
                    reles_states[9] = C_RELE_VOUT;           // Activa el rele 4
                    reles_states[10] = C_RELE_GND;           // Activa el rele 5
                    mcp.digitalWrite(reles[9], C_RELE_VOUT); // Activa el rele 10
                    mcp.digitalWrite(reles[10], C_RELE_GND); // Activa el rele 11
                    break;
                case 11:
                    reles_states[10] = C_RELE_VOUT;           // Activa el rele 11
                    reles_states[11] = C_RELE_GND;            // Activa el rele 12
                    mcp.digitalWrite(reles[10], C_RELE_VOUT); // Activa el rele 11
                    mcp.digitalWrite(reles[11], C_RELE_GND);  // Activa el rele 12
                    break;
                case 12:
                    reles_states[11] = C_RELE_VOUT;           // Activa el rele 12
                    reles_states[12] = C_RELE_GND;            // Activa el rele 13
                    mcp.digitalWrite(reles[11], C_RELE_VOUT); // Activa el rele 12
                    mcp.digitalWrite(reles[12], C_RELE_GND);  // Activa el rele 13
                    break;
                case 13:
                    reles_states[12] = C_RELE_VOUT;           // Activa el rele 13
                    reles_states[13] = C_RELE_GND;            // Activa el rele 14
                    mcp.digitalWrite(reles[12], C_RELE_VOUT); // Activa el rele 13
                    mcp.digitalWrite(reles[13], C_RELE_GND);  // Activa el rele 14
                    break;
                case 14:
                    reles_states[13] = C_RELE_VOUT;           // Activa el rele 14
                    reles_states[14] = C_RELE_GND;            // Activa el rele 15
                    mcp.digitalWrite(reles[13], C_RELE_VOUT); // Activa el rele 14
                    mcp.digitalWrite(reles[14], C_RELE_GND);  // Activa el rele 15
                    break;
                case 15:
                    reles_states[14] = C_RELE_VOUT;           // Activa el rele 15
                    reles_states[15] = C_RELE_GND;            // Activa el rele 16
                    mcp.digitalWrite(reles[14], C_RELE_VOUT); // Activa el rele 15
                    mcp.digitalWrite(reles[15], C_RELE_GND);  // Activa el rele 16
                    break;
                case 16:
                    reles_states[15] = C_RELE_VOUT;           // Activa el rele 16
                    reles_states[16] = C_RELE_GND;            // Activa el rele 17
                    mcp.digitalWrite(reles[15], C_RELE_VOUT); // Activa el rele 16
                    mcp.digitalWrite(reles[16], C_RELE_GND);  // Activa el rele 17
                    break;
                case 17:
                    reles_states[16] = C_RELE_VOUT;           // Activa el rele 17
                    reles_states[17] = C_RELE_GND;            // Activa el rele 18
                    mcp.digitalWrite(reles[16], C_RELE_VOUT); // Activa el rele 17
                    digitalWrite(C_PIN_SW_M18, C_RELE_GND);   // Activa el rele 18
                    break;

                default:
                    break;
                }
            }
            else if (cont_secs % 15 == 0)
            {
                timer_sense_SD.set(500); // 500 ms para leer el SD
            }
            if (cont_secs % 60 == 0)
            {
                //cont_secs = 0;
                cont_min++;
                Serial.println();
                Serial.print("Hora: ");
                Serial.println(cont_hours);
                Serial.print("Minutos: ");
                Serial.println(cont_min);
                Serial.println();
                if (cont_min % 60 == 0)
                {
                    cont_min = 0;
                    cont_hours++;
                    if (cont_hours >= 10)
                    {
                        cont_hours = 0;
                        state_machine_status = C_MCHN_ST_STOP;
                    }
                }
            }
        }
        if (timer_sense_SD.poll() != C_TIMER_NOT_EXPIRED)
        {
            newSDRead = true; // Indica que se debe leer el SD

            Serial.println("Reading SD...");
            while (newSDRead == true)
            {
            }
        }
        if (click_btn_1)
        {
            state_machine_status = C_MCHN_ST_TESTING;
        }
        else if (click_btn_2)
        {
            // state_machine_status = C_MCHN_ST_ERROR;
        }
        else if (click_btn_3)
        {

            // state_machine_status = C_MCHN_ST_STOP;
        }
        else if (click_btn_4)
        {
            state_machine_status = C_MCHN_ST_HOLD;
        }
        break;

    case C_MCHN_ST_ERROR:
        led_1 = false;
        led_2 = true;
        led_3 = false;
        if (click_btn_1)
        {
            state_machine_status = C_MCHN_ST_TESTING;
        }
        else if (click_btn_2)
        {
            state_machine_status = C_MCHN_ST_ERROR;
        }
        else if (click_btn_3)
        {
            state_machine_status = C_MCHN_ST_STOP;
        }
        else if (click_btn_4)
        {
            state_machine_status = C_MCHN_ST_HOLD;
        }
        break;

    case C_MCHN_ST_STOP:
        led_1 = true;
        led_2 = true;
        led_3 = false;
        if (click_btn_1)
        {
            state_machine_status = C_MCHN_ST_TESTING;
        }
        else if (click_btn_2)
        {
            state_machine_status = C_MCHN_ST_ERROR;
        }
        else if (click_btn_3)
        {
            state_machine_status = C_MCHN_ST_STOP;
        }
        else if (click_btn_4)
        {
            state_machine_status = C_MCHN_ST_HOLD;
        }
        break;

    default:
        break;
    }
}
// ----------------------------------------------------------------------
//                              CORE 2
// ----------------------------------------------------------------------
void setup1()
{
    display.begin();
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.print("HOLA");
    display.setCursor(0, 20);
    display.print(digitalRead(C_PIN_TEST_1));
    display.setCursor(0, 40);
    display.print(digitalRead(C_PIN_TEST_2));
    display.display();

    Serial.print(digitalRead(C_PIN_TEST_1));
    Serial.println(digitalRead(C_PIN_TEST_2));
    // Inicializa el UID
    if ((digitalRead(C_PIN_TEST_1) == false) && (digitalRead(C_PIN_TEST_2) == false))
    {
        uid = 0;
        Serial.println("No UID found, initializing to 0");
    }
    else
    {
        Serial.println("Reading UID from EEPROM");
        uid = EEPROM.read(EEPROM_UID_ADDR); // Leer el UID de la EEPROM
    }
    if (uid == 0xFFFF) // Si el UID es 0xFFFF, significa que no se ha escrito
    {
        uid = 0;                            // Inicializa el UID
        EEPROM.write(EEPROM_UID_ADDR, uid); // Guarda el UID en la EEPROM
        EEPROM.commit();                    // Asegura que se guarde en la EEPROM
    }
    Serial.print("UID:");
    Serial.println(uid);

    // Inicializa la tarjeta SD
    while (!SD.begin(C_PIN_SS_SD))
    {
        Serial.println("SD Card initialization failed");
        delay(1000); // Espera 1 segundo antes de reintentar
        digitalWrite(C_PIN_SS_LED, LOW);
        display.clearDisplay();
        display.setCursor(0, 0);
        display.print("SD Card Init");
        display.setCursor(0, 20);
        display.print("Failed");
        display.display();
    }
    InitialiceFiles();
    Serial.println("SD Card initialized successfully");
    WriteSDFiles(cont_rele);
}
void loop1()
{
    // Cada 10 segundos escribiremos en el SD pasando el rele actual.

    if (SPI_Enabled)
    {
        delay(100); // Espera para estabilizar la comunicacion SPI
        switch (state_machine_status)
        {
        case C_MCHN_ST_HOLD:
            display.clearDisplay();
            display.setCursor(0, 0);
            display.print("HOLD");
            display.setCursor(0, 20);
            display.print("Click to start");
            display.display();
            break;
        case C_MCHN_ST_TESTING:
            display.clearDisplay();
            display.setCursor(0, 0);
            display.print("TESTING");
            display.setCursor(0, 20);
            display.print("Rele:");
            display.print(cont_rele + 1);
            display.setCursor(0, 40);
            display.print("Current:");
            display.print(motor_current);
            display.display();
            break;
        case C_MCHN_ST_ERROR:
            display.clearDisplay();
            display.setCursor(0, 0);
            display.print("ERROR");
            display.setCursor(0, 20);
            display.print("Click to reset");
            display.display();
            break;
        case C_MCHN_ST_STOP:
            display.clearDisplay();
            display.setCursor(0, 0);
            display.print("STOP");
            display.setCursor(0, 20);
            display.print("Click to start");
            display.display();
            break;
        default:
            break;
        }
    }
    if (newSDRead)
    {
        WriteSDFiles(cont_rele);
        newSDRead = false;
    }
}
//------------------------------- FUNCIONES ---------------------------------
void UpdateLeds()
{
    digitalWrite(C_PIN_LED1, led_1);
    digitalWrite(C_PIN_LED2, led_2);
    digitalWrite(C_PIN_LED3, led_3);
    // Serial.println("Leds updated");
}
void SenseCurrent()
{
    if (timer_sense.poll(100) != C_TIMER_NOT_EXPIRED)
    {
        motor_current = 0;
        motor_frec_current = 0;
        for (int i = 0; i < C_NSAMPLES; i++)
        {
            motor_frec_current += analogRead(C_PIN_I_FREC);
            motor_current += analogRead(C_PIN_I_MOTOR);
        }

        motor_frec_current = motor_frec_current / C_NSAMPLES;
        motor_current = motor_current / C_NSAMPLES;

        motor_current = motor_current * 3300 / 4096 / 30;
        newSense = true;
    }
}
void LecturaBotones()
{
    prev_boton_1 = actual_boton_1;
    prev_boton_2 = actual_boton_2;
    prev_boton_3 = actual_boton_3;
    prev_boton_4 = actual_boton_4;
    actual_boton_1 = digitalRead(C_PIN_TEST_1);
    actual_boton_2 = digitalRead(C_PIN_TEST_2);
    actual_boton_3 = digitalRead(C_PIN_TEST_3);
    actual_boton_4 = digitalRead(C_PIN_TEST_4);

    if ((actual_boton_1 == true) && (prev_boton_1 == false))
    {
        click_btn_1 = true;
    }
    else
    {
        click_btn_1 = false;
    }
    if ((actual_boton_2 == true) && (prev_boton_2 == false))
    {
        click_btn_2 = true;
    }
    else
    {
        click_btn_2 = false;
    }
    if ((actual_boton_3 == true) && (prev_boton_3 == false))
    {
        click_btn_3 = true;
    }
    else
    {
        click_btn_3 = false;
    }
    if ((actual_boton_4 == true) && (prev_boton_4 == false))
    {
        click_btn_4 = true;
    }
    else
    {
        click_btn_4 = false;
    }
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
void WriteSDFiles(int rele)
{
    SPI_Enabled = false; // Desactivar SPI para evitar conflictos al escribir en el SD
    // Inlcuir una columna en el csv en funcion del rele
    if (rele < 0 || rele > 17)
    {
        Serial.println("Rele fuera de rango. Debe ser entre 0 y 17.");
        return;
    }
    // Abrir el archivo correspondiente al rele

    File dataFile = SD.open(Filenames[rele], FILE_WRITE);
    // Serial.println(dataFile.availableForWrite());
    if (dataFile)
    {
        dataFile.print(++reles_index_samples[rele]); // Escribir el indice del rele
        dataFile.print(";");
        dataFile.print(cont_secs);
        dataFile.print(";");
        dataFile.print(motor_current);
        dataFile.println(";");
        dataFile.close();
        Serial.printf("Datos escritos %d; %d; %d\n", reles_index_samples[rele], cont_secs, motor_current);
        Serial.println("Grabado en " + Filenames[rele]);
    }
    else
    {
        Serial.println("Error al abrir el archivo " + Filenames[rele]);
    }
    t0 = millis(); // Reiniciar el contador de tiempo
    SPI_Enabled = true;
}
void InitialiceFiles()
{
    SPI_Enabled = false;
    for (int i = 0; i < 18; i++)
    {
        if (SD.exists(Filenames[i]))
        {
            SD.remove(Filenames[i]);
        }
    }

    Serial.println("Archivos existentes eliminados.");
    // Inicializar el archivo data.txt
    Serial.println("Inicializando archivo data.txt...");

    // Inicializar los 18 archivos con una cabecera
    for (int i = 0; i < 18; i++)
    {
        File dataFile = SD.open(Filenames[i], FILE_WRITE);
        if (dataFile)
        {
            uid += 1; // Incrementar el UID para cada archivo
            dataFile.println(Filenames[i]);
            dataFile.println(VERSION);
            dataFile.println(T_SAMPLE);
            dataFile.print(VERSION);
            dataFile.print("_");
            dataFile.println(uid);
            dataFile.print("Voltage: ");
            dataFile.println(volts);
            dataFile.println("Index;Timestamp;Current;");
            dataFile.close();
            Serial.println("Archivo " + String(Filenames[i]) + " inicializado.");
            Serial.printf("UID: %d\n", uid);
        }
        else
        {
            Serial.println("Error al abrir el archivo " + String(Filenames[i]));
        }
    }
    EEPROM.write(EEPROM_UID_ADDR, uid); // Guarda el UID en la EEPROM
    EEPROM.commit();                    // Asegura que se guarde en la EEPROM
    SPI_Enabled = true;
}