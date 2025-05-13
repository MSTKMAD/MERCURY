#include <EEPROM.h>
#include <SoftwareSerial.h>

// #define VERSION "0.1.0" // Version of the code

const int adcPin = A1;               // ADC pin
const int pinVout = A3;              // Pin for voltage measurement
const int threshold = 512;           // Threshold value for ADC
const unsigned long interval = 1000; // Interval for measurement (1 second)
const int eepromLogSize = 350;       // EEPROM array size
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
const int eepromControlDirection = eepromEndAddress + 17; // Address for storing the temperature use

const int eepromSize = eepromControlDirection + 1; // Total EEPROM size

const int C_MOTOR_CLOCKWISE = 0; // Motor clockwise
const int C_MOTOR_COUNTERCLOCKWISE = 1; // Motor counterclockwise

SoftwareSerial mySerial(1, 0); // RX, TX

int v_temp = 0;                       // Variable to store the temperature value
int vin = 0;                       // Variable to store the temperature value
int temp = 0;                         // Variable to store the temperature value
int temp_10s_ant = 0;                 // Variable to store the previous temperature value
int count_minutes = 0;                // Variable to store the number of minutes
int count_seconds = 0;                // Variable to store the number of seconds
int count_milis = 0;                  // Variable to store the number of miliseconds
int eepromIndex = eepromStartAddress; // Variable to store the current EEPROM index
int tempMax = 0;                      // Variable to store the maximum temperature
int tempMin = 0;                      // Variable to store the minimum temperature
int eepromErrorMaxTemp = 0;           // Variable to store the maximum temperature error
int eepromErrorSlopeTemp = 0;         // Variable to store the slope temperature error
int eepromErrorVoltaje = 0;           // Variable to store the voltage error
int eepromHours = 0;                  // Variable to store the hours
int eepromTempUse25Less = 0;          // Variable to store the temperature use
int eepromTempUse2530 = 0;            // Variable to store the temperature use
int eepromTempUse3035 = 0;            // Variable to store the temperature use
int eepromTempUse3540 = 0;            // Variable to store the temperature use  
int eepromTempUse4045 = 0;            // Variable to store the temperature use
int eepromTempUse45More = 0;          // Variable to store the temperature use

bool flag_temp_max = false;   // Flag to indicate if the maximum temperature has been reached
bool flag_slope_temp = false; // Flag to indicate if the slope temperature has been reached
bool flag_voltaje = false;    // Flag to indicate if the voltage has been reached

bool motorDirection = C_MOTOR_CLOCKWISE; // Motor direction

void setup()
{
  digitalWrite(4, HIGH); // Encender el motor nada mas iniciar
  timer1_init();

  motorDirection = EEPROM.read(eepromControlDirection); // Read the motor direction from EEPROM
  if (motorDirection == C_MOTOR_CLOCKWISE)
  {
    digitalWrite(0, HIGH); // Set motor direction to clockwise
    motorDirection = C_MOTOR_COUNTERCLOCKWISE; // Set default motor direction to counterclockwise
  }
  else if (motorDirection == C_MOTOR_COUNTERCLOCKWISE)
  {
    digitalWrite(0, LOW); // Set motor direction to counterclockwise
    motorDirection = C_MOTOR_CLOCKWISE; // Set default motor direction to clockwise
  }
  else
  {
    digitalWrite(0, HIGH); // Default to clockwise if invalid value
    motorDirection = C_MOTOR_COUNTERCLOCKWISE; // Set default motor direction to counterclockwise
  }
  EEPROM.write(eepromControlDirection, motorDirection); // Store the motor direction in EEPROM

  while (1)
  {
    /* code */
  }

  // sei();
  pinMode(A1, INPUT);
  pinMode(4, OUTPUT);
  // analogReference(INTERNAL2V56_NO_CAP);
  mySerial.begin(9600);
  mySerial.println(F("Starting..."));
  mySerial.print(F("Version: "));
  mySerial.println(F("0.2.0"));
  // Dump EEPROM contents
  for (int i = 0; i < eepromSize; i++)
  {
    mySerial.print(F("E["));
    mySerial.print(i);
    mySerial.print(F("] = "));
    uint32_t value = EEPROM.read(i);
    if (value == 0xFF) // Check if the EEPROM location is empty
    {
      value = 0; // Set to 0 if empty
      mySerial.println(value);
      if (EEPROM.read(i) != 0x00)
      {
        EEPROM.write(i, value);
      }
    }
    else
    {
      if (i <= eepromEndAddress)
      {
        value = value * 4;                    // Convert back to original ADC value
        value = value * 3000 / 1024;          // Convert to voltage
        mySerial.println((value - 500) / 10); // Convert to temperature in Celsius
      }
      else
      {
        mySerial.println(value);
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
  digitalWrite(4, HIGH);
}

void loop()
{
  if (count_milis >= interval)
  {
    count_milis = 0; // Reset the milliseconds counter

    // Read ADC value
    uint32_t adcValue = 0;
    for (size_t i = 0; i < 8; i++)
    {
      adcValue += analogRead(adcPin);
      delay(10); // Delay to allow ADC to stabilize
    }
    adcValue /= 8; // Average the ADC value

    v_temp = adcValue * 3000 / 1024; // Convert ADC value to voltage
    temp = (v_temp - 500) / 10;      // Convert voltage to temperature in Celsius

    // Check if the temperature exceeds the limit
    if ((temp > temp_limit) && (flag_temp_max == false))
    {
      flag_temp_max = true; // Set the maximum temperature flag
      digitalWrite(4, LOW);
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
        digitalWrite(4, HIGH);
      }
    }

    // Check if the temperature exceeds the maximum or minimum values
    if (temp > tempMax)
    {
      tempMax = temp; // Update the maximum temperature
      if (EEPROM.read(eepromErrorMaxTempAddress) != tempMax)
      {
        EEPROM.write(eepromErrorMaxTempAddress, tempMax); // Store the maximum temperature error count in EEPROM
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
          digitalWrite(4, LOW);
        }
      }
    }
    if (flag_slope_temp)
    {
      if (temp - temp_10s_ant < slope_limit - slope_threshold)
      {
        flag_slope_temp = false; // Reset the slope temperature flag
        digitalWrite(4, HIGH);
      }
    }

    // Store the ADC value in EEPROM every 10min
    if (count_seconds >= 60)
    {
      count_seconds = 0;
      count_minutes++;
      if (count_minutes % 10 == 0)
      {
        //  Store ADC value in EEPROM
        if (EEPROM.read(eepromIndex) != adcValue / 4)
        {
          EEPROM.write(eepromIndex, adcValue / 4); // Store the ADC value in EEPROM
        }
        if ((temp >= 25) && (temp < 30))
        {
          eepromTempUse2530 += 1;
          mySerial.println(eepromTempUse2530);
          EEPROM.write(eepromTempUse2530Address, eepromTempUse2530);
          mySerial.print(F(" 1 "));
        }
        else if ((temp >= 30) && (temp < 35))
        {
          eepromTempUse3035 += 1;
          mySerial.println(eepromTempUse3035);
          EEPROM.write(eepromTempUse3035Address, eepromTempUse3035);
          mySerial.print(F(" 2 "));
        }
        else if ((temp >= 35) && (temp < 40))
        {
          eepromTempUse3540 += 1;
          mySerial.println(eepromTempUse3540);
          EEPROM.write(eepromTempUse3540Address, eepromTempUse3540);
          mySerial.print(F(" 3 "));
        }
        else if ((temp >= 40) && (temp < 45))
        {
          eepromTempUse4045 += 1;
          mySerial.println(eepromTempUse4045);
          EEPROM.write(eepromTempUse4045Address, eepromTempUse4045);
          mySerial.print(F(" 4 "));
        }
        else if (temp >= 45)
        {
          eepromTempUse45More += 1;
          mySerial.println(eepromTempUse45More);
          EEPROM.write(eepromTempUse45MoreAddress, eepromTempUse45More);
          mySerial.print(F(" 5 "));
        }
        else if (temp < 25)
        {
          eepromTempUse25Less += 1;
          mySerial.println(eepromTempUse25Less);
          EEPROM.write(eepromTempUse25LessAddress, eepromTempUse25Less);
        }
        eepromIndex = eepromIndex + 1;
        if (eepromIndex >= eepromLogSize)
        {
          eepromIndex = 0; // Reset the index if it exceeds the EEPROM size
        }
        EEPROM.write(eepromIndexAddress, eepromIndex); // Store the current index in EEPROM
        EEPROM.write(eepromMinAddress, count_minutes); // Store the current index in EEPROM
      }
      if (count_minutes >= 60)
      {
        count_minutes = 0;
        eepromHours = eepromHours + 1;                 // Increment the hour count
        EEPROM.write(eepromHoursAddress, eepromHours); // Store the hour count in EEPROM
      }
    }

    mySerial.print(eepromHours);
    mySerial.print(F(" : "));
    mySerial.print(count_minutes);
    mySerial.print(F(" : "));
    mySerial.print(count_seconds);
    mySerial.print(F(" -> "));
    mySerial.println(temp);
  }
}

void timer1_init()
{
  // CTC mode
  TCCR1 = (1 << CTC1);

  // Prescaler 64
  TCCR1 |= (1 << CS11) | (1 << CS10);

  // Comparación cada 10 ms: OCR1C = (0.01 * 8e6 / 64) - 1 = 1249
  OCR1C = 1249;

  // Habilita interrupción por comparación
  TIMSK |= (1 << OCIE1A);

  // Limpia contador
  TCNT1 = 0;
}
ISR(TIMER1_COMPA_vect)
{
  count_milis += 10; // Incrementa el contador de milisegundos
}