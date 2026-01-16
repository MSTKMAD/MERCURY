#include <EEPROM.h>
#include <SoftwareSerial.h>

// #define VERSION "0.1.0" // Version of the code

const int pinVtemp = A1;          // ADC pin
const int pinIout = A2;           // Pin for current measurement
const int pinVout = A3;           // Pin for voltage measurement
const int pinMotorDirection1 = 0; // Pin for motor direction control
const int pinMotorDirection2 = 1; // Pin for motor direction control
const int pinTx = 0;              // Pin for TX
const int pinRx = 1;              // Pin for RX

const int threshold = 512;           // Threshold value for ADC
const unsigned long interval = 1000; // Interval for measurement (1 second)
const int eepromLogSize = 1;         // EEPROM array size
const int temp_limit = 40;           // Temperature limit in Celsius
const int slope_limit = 5;           // Slope limit for temperature change
const int temp_threshold = 5;        // Temperature threshold for slope detection
const int slope_threshold = 3;       // Slope threshold for slope detection

const int eepromStartAddress = 0;                                    // Start address for EEPROM storage
const int eepromEndAddress = eepromStartAddress + eepromLogSize - 1; // End address for EEPROM storage
const int eepromIndexAddress = eepromEndAddress + 1;                 // Address for storing the current index
const int eepromIndexTempMaxAddress = eepromEndAddress + 2;          // Address for storing the maximum temperature
const int eepromIndexTempMinAddress = eepromEndAddress + 3;          // Address for storing the minimum temperature
const int eepromHoursAddress = eepromEndAddress + 4;                 // Address for storing the hours
const int eepromMinAddress = eepromEndAddress + 5;                   // Address for storing the minimum temperature
const int eepromErrorMaxTempAddress = eepromEndAddress + 6;          // Address for storing the maximum temperature error
const int eepromErrorSlopeTempAddress = eepromEndAddress + 7;        // Address for storing the slope temperature error
const int eepromErrorVoltajeAddress = eepromEndAddress + 8;          // Address for storing the voltage error

const int eepromTempUse2530Address = eepromEndAddress + 10;   // Address for storing the temperature use
const int eepromTempUse3035Address = eepromEndAddress + 11;   // Address for storing the temperature use
const int eepromTempUse3540Address = eepromEndAddress + 12;   // Address for storing the temperature use
const int eepromTempUse4045Address = eepromEndAddress + 13;   // Address for storing the temperature use
const int eepromTempUse45MoreAddress = eepromEndAddress + 15; // Address for storing the temperature use
const int eepromTempUse25LessAddress = eepromEndAddress + 16; // Address for storing the temperature use
const int eepromControlDirection = eepromEndAddress + 17;     // Address for storing the temperature use

// EEPROM for Voltage usage
const int eepromVoltageUsageStartAddress = eepromControlDirection + 1;                                // Start address for voltage usage
const int eepromVoltageUsageAddress = eepromVoltageUsageStartAddress;                                 // Address for storing the voltage usage
const int eepromVoltageUsageSize = 18;                                                                // Size of the voltage usage array
const int eepromVoltageUsageEndAddress = eepromVoltageUsageStartAddress + eepromVoltageUsageSize - 1; // End address for voltage usage

const int eepromSize = eepromVoltageUsageEndAddress + 1; // Total EEPROM size

const int C_MOTOR_CLOCKWISE = 0;        // Motor clockwise
const int C_MOTOR_COUNTERCLOCKWISE = 1; // Motor counterclockwise

const int C_OC_LIMIT = 310;             // Overcurrent limit(1A). Visen = 1V/A. ADC = 1024/3.3V * 1000 mA = 310.
const int C_VOLTAGE_LIMIT = 14000;      // Voltage limit in mV
const int C_CONSUM_CURRENT_LIMIT = 155; // Consumption current limit in ADC units (500mA) ADC = 1024/3.3V * 500 mA = 155

// Temporizadores de las protecciones.
const int C_FREQ_MES = 10;           // Frecuencia de medición en ms
const int C_TEMP_MAX_TIME = 1000;    // Tiempo para la protección de temperatura máxima
const int C_SLOPE_TIME = 1000;       // Tiempo para la protección de pendiente de temperatura
const int C_VOLTAGE_TIME = 500;      // Tiempo para la protección de voltaje
const int C_OC_TIME = 100;           // Tiempo para la protección de sobrecorriente
const int C_CONSUM_CURR_TIME = 2000; // Tiempo para la protección de consumo de corriente

const int C_COUNT_MAX_VOLTAGE = C_VOLTAGE_TIME / C_FREQ_MES;       // Contador máximo para la protección de voltaje
const int C_COUNT_MAX_OC = C_OC_TIME / C_FREQ_MES;                 // Contador máximo para la protección de sobrecorriente
const int C_COUNT_MAX_SLOPE = C_SLOPE_TIME / C_FREQ_MES;           // Contador máximo para la protección de pendiente de temperatura
const int C_COUNT_MAX_TEMP = C_TEMP_MAX_TIME / C_FREQ_MES;         // Contador máximo para la protección de temperatura máxima
const int C_COUNT_MAX_CONS_CURR = C_CONSUM_CURR_TIME / C_FREQ_MES; // Contador máximo para la protección de consumo de corriente



int cont_oc_time = 0;        // Contador de tiempo para la protección de sobrecorriente
int cont_voltage_time = 0;   // Contador de tiempo para la protección de voltaje
int cont_slope_time = 0;     // Contador de tiempo para la protección de pendiente de temperatura
int cont_temp_max_time = 0;  // Contador de tiempo para la protección de temperatura máxima
int cont_cons_curr_time = 0; // Contador de tiempo para la protección de consumo de corriente

SoftwareSerial mySerial(1, 0); // RX, TX

int v_temp = 0;                        // Variable to store the temperature value
int vin = 0;                           // Variable to store the temperature value
int temp = 0;                          // Variable to store the temperature value
int temp_10s_ant = 0;                  // Variable to store the previous temperature value
int count_minutes = 0;                 // Variable to store the number of minutes
int count_seconds = 0;                 // Variable to store the number of seconds
uint16_t count_milis = 0;              // Variable to store the number of miliseconds
int eepromIndex = eepromStartAddress;  // Variable to store the current EEPROM index
int tempMax = 0;                       // Variable to store the maximum temperature
int tempMin = 0xFF;                    // Variable to store the minimum temperature
int eepromErrorMaxTemp = 0;            // Variable to store the maximum temperature error
int eepromErrorSlopeTemp = 0;          // Variable to store the slope temperature error
int eepromErrorVoltaje = 0;            // Variable to store the voltage error
int eepromHours = 0;                   // Variable to store the hours
int eepromTempUse25Less = 0;           // Variable to store the temperature use
int eepromTempUse2530 = 0;             // Variable to store the temperature use
int eepromTempUse3035 = 0;             // Variable to store the temperature use
int eepromTempUse3540 = 0;             // Variable to store the temperature use
int eepromTempUse4045 = 0;             // Variable to store the temperature use
int eepromTempUse45More = 0;           // Variable to store the temperature use
int millis_check = 0;                  // Variable to store the milliseconds for checking
uint32_t current_previous_max_adc = 0; // Variable to store the previous maximum ADC value for moving average
uint32_t current_max_adc = 0;          // Variable to store the current maximum ADC value for moving average
uint32_t adcValue = 0;                 // Variable to store the ADC value

int Voltage_Usage[18];

uint32_t v_out = 0; // Variable to store the voltage value

bool flag_temp_max = false;     // Flag to indicate if the maximum temperature has been reached
bool flag_slope_temp = false;   // Flag to indicate if the slope temperature has been reached
bool flag_voltaje = false;      // Flag to indicate if the voltage has been reached
bool flag_over_current = false; // Flag to indicate if the  current has been reached
bool flag_cons_current = false; // Flag to indicate if the current has been reached

bool eeprom_motorDirection = C_MOTOR_CLOCKWISE; // Motor direction
bool local_motorDirection = C_MOTOR_CLOCKWISE;  // Motor direction

bool motorMode = false;    // Motor mode (true: normal, false: serial)
bool adcArrayFull = false; // Flag to indicate if the ADC array is full

void setup()
{
  ChangePrescalerClockDivider(); // Change the clock prescaler to 8MHz
  timer1_init();
  eeprom_motorDirection = EEPROM.read(eepromControlDirection); // Read the motor direction from EEPROM
  local_motorDirection = eeprom_motorDirection;                // Set the local motor direction to the EEPROM value

  // Lectura del pin A3 para control del voltaje
  pinMode(pinVtemp, INPUT);
  pinMode(pinIout, INPUT);
  pinMode(pinVout, INPUT);
  pinMode(pinMotorDirection1, OUTPUT);
  pinMode(pinMotorDirection2, OUTPUT);
  digitalWrite(pinMotorDirection2, LOW); // Apagar el motor nada mas iniciar
  digitalWrite(pinMotorDirection1, LOW); // Apagar el motor nada mas iniciar
  for (int i = 0; i < 8; i++)
  {
    v_out += analogRead(pinVout); // Dummy read to stabilize the ADC
    delay(10);                    // Delay to allow ADC to stabilize
  }

  v_out /= 8;                             // Average the readings
  v_out = v_out * 3300 / 1024 * 208 / 39; // Convert ADC value to voltage

  if (v_out > 4000)
  {
    motorMode = true; // Normal motor mode
    if (eeprom_motorDirection == C_MOTOR_CLOCKWISE)
    {
      digitalWrite(pinMotorDirection2, LOW);            // Encender el motor nada mas iniciar
      digitalWrite(pinMotorDirection1, HIGH);           // Encender el motor nada mas iniciar
      eeprom_motorDirection = C_MOTOR_COUNTERCLOCKWISE; // Set default motor direction to counterclockwise
    }
    else if (eeprom_motorDirection == C_MOTOR_COUNTERCLOCKWISE)
    {
      digitalWrite(pinMotorDirection1, LOW);     // Encender el motor nada mas iniciar
      digitalWrite(pinMotorDirection2, HIGH);    // Encender el motor nada mas iniciar
      eeprom_motorDirection = C_MOTOR_CLOCKWISE; // Set default motor direction to clockwise
    }
    else
    {
      digitalWrite(pinMotorDirection2, LOW);            // Encender el motor nada mas iniciar
      digitalWrite(pinMotorDirection1, HIGH);           // Encender el motor nada mas iniciar
      eeprom_motorDirection = C_MOTOR_COUNTERCLOCKWISE; // Set default motor direction to counterclockwise
    }
    EEPROM.write(eepromControlDirection, eeprom_motorDirection); // Store the motor direction in EEPROM
  }
  else
  {
    motorMode = false;                     // Serial motor mode
    digitalWrite(pinMotorDirection2, LOW); // Apagar el motor nada mas iniciar
    digitalWrite(pinMotorDirection1, LOW); // Apagar el motor nada mas iniciar
  }

  // analogReference(INTERNAL2V56_NO_CAP);
  /*
  mySerial.begin(9600);
  mySerial.println(F("Starting..."));
  mySerial.print(F("Version: "));
  mySerial.println(F("0.2.0"));
  */
  // Dump EEPROM contents
  for (int i = 0; i < eepromSize; i++)
  {
    if (i == eepromVoltageUsageAddress)
    {
      if (!motorMode)
      {
        // mySerial.println("-----------");
      }
    }
    if (!motorMode)
    {
      // mySerial.print(F("E["));
      // mySerial.print(i);
      // mySerial.print(F("] = "));
    }
    uint32_t value = EEPROM.read(i);
    if (value == 0xFF) // Check if the EEPROM location is empty
    {
      value = 0; // Set to 0 if empty
      if (EEPROM.read(i) != 0x00)
      {
        EEPROM.write(i, value);
      }
    }
    else
    {
      if (i <= eepromEndAddress)
      {
        value = value * 4;           // Convert back to original ADC value
        value = value * 3300 / 1024; // Convert to voltage
        if (!motorMode)
        {
          // mySerial.println((value - 500) / 10); // Convert to temperature in Celsius
        }
      }
      else if (i >= eepromVoltageUsageAddress && i < eepromVoltageUsageAddress + 18)
      {
        if (!motorMode)
        {
          // mySerial.println(EEPROM.read(i));
        }
        Voltage_Usage[i - eepromVoltageUsageAddress] = EEPROM.read(i);
      }
      else
      {
        if (!motorMode)
        {
          // mySerial.println(value);
        }
      }
    }
    switch (i)
    {
    case eepromIndexAddress:
      eepromIndex = EEPROM.read(i); // Read the current index from EEPROM
      break;
    case eepromIndexTempMaxAddress:
      tempMax = EEPROM.read(i); // Read the maximum temperature from EEPROM
      break;
    case eepromIndexTempMinAddress:
      tempMin = EEPROM.read(i); // Read the minimum temperature from EEPROM
      break;
    case eepromHoursAddress:
      eepromHours = EEPROM.read(i); // Read the hours from EEPROM
      break;
    case eepromMinAddress:
      count_minutes = EEPROM.read(i); // Read the minutes from EEPROM
      break;
    case eepromErrorMaxTempAddress:
      eepromErrorMaxTemp = EEPROM.read(i); // Read the maximum temperature error from EEPROM
      break;
    case eepromErrorSlopeTempAddress:
      eepromErrorSlopeTemp = EEPROM.read(i); // Read the slope temperature error from EEPROM
      break;
    case eepromErrorVoltajeAddress:
      eepromErrorVoltaje = EEPROM.read(i); // Read the voltage error from EEPROM
      break;
    case eepromTempUse2530Address:
      eepromTempUse2530 = EEPROM.read(i);
      break;
    case eepromTempUse3035Address:
      eepromTempUse3035 = EEPROM.read(i);
      break;
    case eepromTempUse3540Address:
      eepromTempUse3540 = EEPROM.read(i);
      break;
    case eepromTempUse4045Address:
      eepromTempUse4045 = EEPROM.read(i);
      break;
    case eepromTempUse45MoreAddress:
      eepromTempUse45More = EEPROM.read(i);
      break;
    case eepromTempUse25LessAddress:
      eepromTempUse25Less = EEPROM.read(i);
      break;

    default:
      break;
    }
  }
}

void loop()
{
  if (motorMode)
  {
    // ---------------------------------- Main loop every 1 second ----------------------------------
    if (count_milis >= interval)
    {
      /*
      mySerial.print("millis:");
      mySerial.print((millis() - millis_check));
      mySerial.print("-> timer:");
      mySerial.println(count_milis);
      millis_check = millis();
      */
      count_milis = 0; // Reset the milliseconds counter

      // ----------------------------------   Read ADC value for Temperature   ----------------------------------

      // Check if the temperature exceeds the limit
      if ((temp > temp_limit) && (flag_temp_max == false))
      {
        flag_temp_max = true;                        // Set the maximum temperature flag
        eepromErrorMaxTemp = eepromErrorMaxTemp + 1; // Increment the maximum temperature error count
        if (EEPROM.read(eepromErrorMaxTempAddress) != eepromErrorMaxTemp)
        {
          EEPROM.write(eepromErrorMaxTempAddress, eepromErrorMaxTemp); // Store the maximum temperature error count in EEPROM
        }
      }
      if (flag_temp_max)
      {
        if (temp < temp_limit - temp_threshold)
        {
          flag_temp_max = false; // Reset the maximum temperature flag
        }
      }
      // Check if the temperature exceeds the maximum or minimum values registered
      if (temp > tempMax)
      {
        tempMax = temp; // Update the maximum temperature
        if (EEPROM.read(eepromIndexTempMaxAddress) != tempMax)
        {
          EEPROM.write(eepromIndexTempMaxAddress, tempMax); // Store the maximum temperature error count in EEPROM
        }
      }
      if (temp < tempMin)
      {
        tempMin = temp; // Update the minimum temperature
        if (EEPROM.read(eepromIndexTempMinAddress) != tempMin)
        {
          EEPROM.write(eepromIndexTempMinAddress, tempMin);
        }
      }
      // ---------------------------------- Cont Seconds and Minutes ----------------------------------
      count_seconds++;
      if (count_seconds % 10 == 0)
      {
        temp_10s_ant = temp; // Store the temperature value every 10 seconds
        if ((temp - temp_10s_ant > 0) && (flag_slope_temp == false))
        {
          if (temp - temp_10s_ant > slope_limit)
          {
            flag_slope_temp = true;                          // Set the slope temperature flag
            eepromErrorSlopeTemp = eepromErrorSlopeTemp + 1; // Increment the slope temperature error count
            if (EEPROM.read(eepromErrorSlopeTempAddress) != eepromErrorSlopeTemp)
            {
              EEPROM.write(eepromErrorSlopeTempAddress, eepromErrorSlopeTemp); // Store the slope temperature error count in EEPROM
            }
          }
        }
      }
      if (flag_slope_temp)
      {
        if (temp - temp_10s_ant < slope_limit - slope_threshold)
        {
          flag_slope_temp = false; // Reset the slope temperature flag
        }
      }

      // ---------------------------------- Store the ADC value in EEPROM every 10min ----------------------------------
      if (count_seconds >= 60)
      {
        count_seconds = 0;
        count_minutes++;
        if (count_minutes % 10 == 0)
        {
          if ((temp >= 25) && (temp < 30))
          {
            eepromTempUse2530 += 1;
            // mySerial.println(eepromTempUse2530);
            EEPROM.write(eepromTempUse2530Address, eepromTempUse2530);
            // mySerial.print(F(" 1 "));
          }
          else if ((temp >= 30) && (temp < 35))
          {
            eepromTempUse3035 += 1;
            // mySerial.println(eepromTempUse3035);
            EEPROM.write(eepromTempUse3035Address, eepromTempUse3035);
            // mySerial.print(F(" 2 "));
          }
          else if ((temp >= 35) && (temp < 40))
          {
            eepromTempUse3540 += 1;
            // mySerial.println(eepromTempUse3540);
            EEPROM.write(eepromTempUse3540Address, eepromTempUse3540);
            // mySerial.print(F(" 3 "));
          }
          else if ((temp >= 40) && (temp < 45))
          {
            eepromTempUse4045 += 1;
            // mySerial.println(eepromTempUse4045);
            EEPROM.write(eepromTempUse4045Address, eepromTempUse4045);
            // mySerial.print(F(" 4 "));
          }
          else if (temp >= 45)
          {
            eepromTempUse45More += 1;
            // mySerial.println(eepromTempUse45More);
            EEPROM.write(eepromTempUse45MoreAddress, eepromTempUse45More);
            // mySerial.print(F(" 5 "));
          }
          else if (temp < 25)
          {
            eepromTempUse25Less += 1;
            // mySerial.println(eepromTempUse25Less);
            EEPROM.write(eepromTempUse25LessAddress, eepromTempUse25Less);
          }

          EEPROM.write(eepromMinAddress, count_minutes); // Store the current index in EEPROM

          uint16_t index_voltage = (v_out - 4000) / 500; // Calculate the index for the voltage usage array
          if (index_voltage >= 18)
          {
            index_voltage = 17; // Ensure the index does not exceed the array size
          }
          Voltage_Usage[index_voltage]++;
          EEPROM.write(eepromVoltageUsageStartAddress + index_voltage, Voltage_Usage[index_voltage]); // Store the voltage usage in EEPROM
        }
        if (count_minutes >= 60)
        {
          count_minutes = 0;
          eepromHours = eepromHours + 1;                 // Increment the hour count
          EEPROM.write(eepromHoursAddress, eepromHours); // Store the hour count in EEPROM
        }
      }

      // ---------------------------------- Logging to Serial ----------------------------------
      /*
          mySerial.print(F(" ST: "));
          mySerial.print(flag_slope_temp);
          mySerial.print(F(" TM: "));
          mySerial.print(flag_temp_max);
          mySerial.print(F(" V: "));
          mySerial.print(flag_voltaje);
          mySerial.print(F(" -----  "));
          mySerial.print(eepromHours);
          mySerial.print(F(" : "));
          mySerial.print(count_minutes);
          mySerial.print(F(" : "));
          mySerial.print(count_seconds);
          mySerial.print(F(" -> "));
          mySerial.print(F("T:"));
          mySerial.print(temp);
          mySerial.print(F(", V:"));
          mySerial.println(v_out);
          */
    }
    // ---------------------------------- Read ADC values every 50ms ----------------------------------
    if (count_milis % C_FREQ_MES == 0)
    {
      // Lectura de los valores del ADC
      v_out = 0;           // Reset the voltage output variable
      adcValue = 0;        // Reset the ADC value
      current_max_adc = 0; // Reset the current maximum ADC value

      v_out += analogRead(pinVout);          // Read the ADC value for voltage
      adcValue += analogRead(pinVtemp);      // Read the ADC value for temperature
      current_max_adc = analogRead(pinIout); // Read the ADC value for current

      v_out = v_out * 3300 / 1024 * 208 / 39; // Convert ADC value to voltage

      v_temp = adcValue * 3300 / 1024; // Convert ADC value to voltage
      temp = (v_temp - 500) / 10;      // Convert voltage to temperature in Celsius

      // Control del Voltaje de Entrada
      if (v_out > 14000)
      {
        flag_voltaje = true; // Set the voltage flag
      }
      else
      {
        flag_voltaje = false; // Reset the voltage flag
      }

      // Control de la Corriente de Salida
      if (current_max_adc > C_OC_LIMIT)
      {
        cont_oc_time++;
        if (cont_oc_time >= C_COUNT_MAX_OC)
        {
          flag_over_current = true; // Turn off the motor if overcurrent is detected
        }
      }
      else if (current_max_adc > C_CONSUM_CURRENT_LIMIT)
      {
        cont_cons_curr_time++;
        if (cont_cons_curr_time >= C_COUNT_MAX_CONS_CURR)
        {
          flag_cons_current = true; // Turn off the motor if consumption current limit is exceeded
        }
      }
      else
      {
        cont_cons_curr_time--;
        if (cont_cons_curr_time <= 0)
        {
          flag_cons_current = false; // Turn off the motor if consumption current limit is exceeded
          cont_cons_curr_time = 0;
        }
        cont_oc_time--;
        if (cont_oc_time <= 0)
        {
          flag_over_current = false; // Turn on the motor if no overcurrent is detected
          cont_oc_time = 0;
        }
      }

      // ---------------------------------- Control Salida ----------------------------------
      if (flag_slope_temp || flag_temp_max || flag_voltaje || flag_over_current || flag_cons_current)
      {
        ApagarMotor(); // Turn off the motor if any of the flags are set
      }
      else
      {
        EncenderMotor(); // Turn on the motor if no flags are set
      }
    }
  }
  else
  {
    ApagarMotor(); // Turn off the motor in serial mode
  }
}
// Cambia el valor del prescaler del reloj principal para tener un reloj de 8MHz en el attiny85
void ChangePrescalerClockDivider()
{
  // Cambia el prescaler del reloj principal a 8MHz
  // Desbloquea el registro de prescaler
  CLKPR = (1 << CLKPCE);
  // Cambia el prescaler a 1 (8MHz)
  CLKPR = 0;
}
void timer1_init()
{
  TCCR1 = (1 << 7);                        // Modo CTC
  TCCR1 |= (1 << 2) | (1 << 1) | (1 << 0); // Prescaler 64
  OCR1C = 124;                             // 125,000 / (124 + 1) = 1000 Hz
  TIMSK |= (1 << 6);                       // Habilita interrupción por OCR1A
  TCNT1 = 0;
}
ISR(TIMER1_COMPA_vect)
{
  count_milis += 1; // Incrementa el contador de milisegundos
}

void EncenderMotor()
{
  if (local_motorDirection == C_MOTOR_CLOCKWISE)
  {
    digitalWrite(pinMotorDirection2, LOW);  // Encender el motor en sentido horario
    digitalWrite(pinMotorDirection1, HIGH); // Encender el motor en sentido horario
  }
  else if (local_motorDirection == C_MOTOR_COUNTERCLOCKWISE)
  {
    digitalWrite(pinMotorDirection1, LOW);  // Encender el motor en sentido antihorario
    digitalWrite(pinMotorDirection2, HIGH); // Encender el motor en sentido antihorario
  }
}
void ApagarMotor()
{
  digitalWrite(pinMotorDirection2, LOW); // Apagar el motor
  digitalWrite(pinMotorDirection1, LOW); // Apagar el motor
}

// la funcion readAndFilterADC  almacena en un array los valores leidos por el ADC y cuando el array esta lleno, activa un booleano para indicar que el array esta lleno
// y devuelve el valor MAXIMO de las ultimas N lecturas del ADC. El booleano tiene que salir de la funcion para ser usado en el loop principal.
bool readAndFilterADC(uint32_t &maxValue)
{
  const int numReadings = 12;            // Number of readings to store
  static uint32_t readings[numReadings]; // Array to store the readings
  static int readIndex = 0;              // Current index in the readings array
  static bool arrayFull = false;         // Flag to indicate if the array is full

  // Read from the ADC
  readings[readIndex] = analogRead(pinIout);

  // Advance to the next index
  readIndex++;

  // If we've reached the end of the array, wrap around and set the flag
  if (readIndex >= numReadings)
  {
    readIndex = 0;
    arrayFull = true;
  }

  // If the array is full, return the maximum value
  if (arrayFull)
  {
    maxValue = readings[0];
    for (int i = 1; i < numReadings; i++)
    {
      if (readings[i] > maxValue)
      {
        maxValue = readings[i];
      }
    }
    maxValue = maxValue;
    return true;
  }
  else
  {
    return false;
  }
}