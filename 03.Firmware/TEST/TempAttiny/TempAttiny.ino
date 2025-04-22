#include <EEPROM.h>
#include <SoftwareSerial.h>

// #define VERSION "0.1.0" // Version of the code

const int adcPin = A1;               // ADC pin
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

const int eepromSize = eepromTempUse25LessAddress + 1; // Total EEPROM size

SoftwareSerial mySerial(1, 0); // RX, TX

int v_temp = 0;                       // Variable to store the temperature value
float temp = 0;                       // Variable to store the temperature value
float temp_10s_ant = 0;               // Variable to store the previous temperature value
int count_minutes = 0;                // Variable to store the number of minutes
int count_seconds = 0;                // Variable to store the number of seconds
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

void setup()
{
  digitalWrite(4, HIGH); // Encender el motor nada mas iniciar
  pinMode(A1, INPUT);
  pinMode(4, OUTPUT);
  // analogReference(INTERNAL2V56_NO_CAP);
  mySerial.begin(9600);
  mySerial.println("Starting...");
  mySerial.print("Version: ");
  mySerial.println("0.2.0");
   // Dump EEPROM contents
  for (int i = 0; i < eepromSize; i++)
  {
    mySerial.print("EEPROM[");
    mySerial.print(i);
    mySerial.print("] = ");
    uint32_t value = EEPROM.read(i);
    if (value == 0xFF) // Check if the EEPROM location is empty
    {
      value = 0; // Set to 0 if empty
      mySerial.println(value);
      EEPROM.write(i, value);
    }
    else
    {
      if (i <= eepromEndAddress)
      {
        value = value * 4;                           // Convert back to original ADC value
        value = value * 3000 / 1024;                 // Convert to voltage
        mySerial.println(((float)value - 500) / 10); // Convert to temperature in Celsius
      }
      else
      {
        mySerial.println(value);
        /* code */
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
  static unsigned long lastMillis = 0;
  if (millis() - lastMillis >= interval)
  {
    lastMillis = millis();

    // Read ADC value
    uint32_t adcValue = 0;
    for (size_t i = 0; i < 8; i++)
    {
      adcValue += analogRead(adcPin);
      delay(10); // Delay to allow ADC to stabilize
    }
    adcValue /= 8; // Average the ADC value

    v_temp = adcValue * 3000 / 1024;   // Convert ADC value to voltage
    temp = ((float)v_temp - 500) / 10; // Convert voltage to temperature in Celsius

    // Check if the temperature exceeds the limit
    if ((temp > temp_limit) && (flag_temp_max == false))
    {
      flag_temp_max = true; // Set the maximum temperature flag
      digitalWrite(4, LOW);
      eepromErrorMaxTemp = eepromErrorMaxTemp + 1;                 // Increment the maximum temperature error count
      EEPROM.write(eepromErrorMaxTempAddress, eepromErrorMaxTemp); // Store the maximum temperature error count in EEPROM
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
      tempMax = temp;                                   // Update the maximum temperature
      EEPROM.write(eepromIndexTempMaxAddress, tempMax); // Store the maximum temperature in EEPROM
    }
    if (temp < tempMin)
    {
      tempMin = temp;                                   // Update the minimum temperature
      EEPROM.write(eepromIndexTempMinAddress, tempMin); // Store the minimum temperature in EEPROM
    }

    count_seconds++;
    if (count_seconds % 10 == 0)
    {
      temp_10s_ant = temp; // Store the temperature value every 10 seconds
      if ((temp - temp_10s_ant > 0) && (flag_slope_temp == false))
      {
        if (temp - temp_10s_ant > slope_limit)
        {
          flag_slope_temp = true;                                          // Set the slope temperature flag
          eepromErrorSlopeTemp = eepromErrorSlopeTemp + 1;                 // Increment the slope temperature error count
          EEPROM.write(eepromErrorSlopeTempAddress, eepromErrorSlopeTemp); // Store the slope temperature error count in EEPROM
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
        EEPROM.write(eepromIndex, adcValue / 4); // Store the ADC value in EEPROM
        if ((temp >= 25.0) && (temp < 30.0))
        {
          eepromTempUse2530 += 1;
          mySerial.println(eepromTempUse2530);
          EEPROM.write(eepromTempUse2530Address, eepromTempUse2530);
          mySerial.print(" 1 ");
        }
        else if ((temp >= 30.0) && (temp < 35.0))
        {
          eepromTempUse3035 += 1;
          mySerial.println(eepromTempUse3035);
          EEPROM.write(eepromTempUse3035Address, eepromTempUse3035);
          mySerial.print(" 2 ");
        }
        else if ((temp >= 35.0) && (temp < 40.0))
        {
          eepromTempUse3540 += 1;
          mySerial.println(eepromTempUse3540);
          EEPROM.write(eepromTempUse3540Address, eepromTempUse3540);
          mySerial.print(" 3 ");
        }
        else if ((temp >= 40.0) && (temp < 45.0))
        {
          eepromTempUse4045 += 1;
          mySerial.println(eepromTempUse4045);
          EEPROM.write(eepromTempUse4045Address, eepromTempUse4045);
          mySerial.print(" 4 ");
        }
        else if (temp >= 45.0)
        {
          eepromTempUse45More += 1;
          mySerial.println(eepromTempUse45More);
          EEPROM.write(eepromTempUse45MoreAddress, eepromTempUse45More);
          mySerial.print(" 5 ");
        }
        else if (temp < 25.0)
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
    mySerial.print(" : ");
    mySerial.print(count_minutes);
    mySerial.print(" : ");
    mySerial.print(count_seconds);
    mySerial.print(" -> ");
    mySerial.println(temp);
  }
}
