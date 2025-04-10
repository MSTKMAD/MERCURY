#include <EEPROM.h>
#include <SoftwareSerial.h>

const int adcPin = A1;               // ADC pin
const int threshold = 512;           // Threshold value for ADC
const int eepromSize = 200;          // EEPROM array size
const unsigned long interval = 1000; // Interval for measurement (1 second)
const int temp_limit = 30;           // Temperature limit in Celsius

SoftwareSerial mySerial(3, 4); // RX, TX

int v_temp = 0;        // Variable to store the temperature value
float temp = 0;        // Variable to store the temperature value
int count_minutes = 0; // Variable to store the number of minutes
int count_seconds = 0; // Variable to store the number of seconds
int eepromIndex = 0;

void setup()
{
  pinMode(A1, INPUT);
  pinMode(1, OUTPUT);
  // analogReference(INTERNAL2V56_NO_CAP);
  mySerial.begin(9600);

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
    }
    else
    {
      value = value * 4;           // Convert back to original ADC value
      mySerial.print(value);
      mySerial.print(" , ");
      value = value * 3300 / 1024; // Convert to voltage
      mySerial.print(value);
      mySerial.print(" , ");
      mySerial.println(((float)value - 500) / 10);// Convert to temperature in Celsius
    }
  }
  digitalWrite(1, HIGH);
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

    v_temp = adcValue * 3300 / 1024;   // Convert ADC value to voltage
    temp = ((float)v_temp - 500) / 10; // Convert voltage to temperature in Celsius
    if (temp > temp_limit)
    {
      digitalWrite(1, LOW);
    }
    else
    {
      digitalWrite(1, HIGH);
    }

    count_seconds++;
    if (count_seconds >= 60)
    {
      count_seconds = 0;
      count_minutes++;
      if (count_minutes >= 1)
      {
        //  Store ADC value in EEPROM
        eepromIndex = eepromIndex + 1;
        EEPROM.write(eepromIndex, adcValue / 4); // Store the ADC value in EEPROM
        if (eepromIndex >= eepromSize)
        {
          eepromIndex = 0; // Reset the index if it exceeds the EEPROM size
        }
        count_minutes = 0;
      }
    }
    mySerial.print(count_minutes);
    mySerial.print(" : ");
    mySerial.print(count_seconds);
    mySerial.print(" -> ");
    mySerial.print(adcValue);
    mySerial.print(" , ");
    mySerial.print(v_temp);
    mySerial.print(" , ");
    mySerial.println(temp);
  }
}
