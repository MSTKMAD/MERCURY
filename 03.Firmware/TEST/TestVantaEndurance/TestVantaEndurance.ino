#include <MilliTimer.h>

MilliTimer timerSeconds;

const uint8_t pin3 = 3;    // Pin que se apagará
const uint8_t pin4 = 4;    // Pin que hará toggle
const uint8_t pinADC = A1; // Pin que hará toggle

const int c_secIn8hours = 60*15; // 8 horas en segundos
// const int c_secIn8hours = 2; // 8 horas en segundos
const int c_15sec = 15; // 15 segundos en milisegundos
// const int c_15sec = 1; // 15 segundos en milisegundos
bool is15SecondsActive = false;
uint32_t cont_secs = 0; // Contador de segundos

void setup()
{
    pinMode(pin3, OUTPUT);
    pinMode(pin4, OUTPUT);

    digitalWrite(pin3, LOW); // Inicialmente encendido
    /*
    digitalWrite(pin4, LOW);  // Inicialmente apagado
    digitalWrite(1, HIGH);
    
    int vin = 0; // Variable para almacenar el valor de la lectura del pin A3
    for (int i = 0; i < 8; i++)
    {
        digitalWrite(1, LOW);
        vin += analogRead(pinADC); // Dummy read to stabilize the ADC
        delay(10);                 // Delay to allow ADC to stabilize
        digitalWrite(1, HIGH);
    }
    vin /= 8; // Promedio de la lectura del pin A3
    */
    timerSeconds.set(1000); // 8 horas en milisegundos
}

void loop()
{
    if (timerSeconds.poll() != C_TIMER_NOT_EXPIRED)
    {
        cont_secs++;
        if (is15SecondsActive)
        {
            if (cont_secs >= c_15sec) // Si han pasado 15 segundos
            {
                digitalWrite(pin3, LOW); // Enciende el pin3
                cont_secs = 0;             // Reinicia el contador de segundos
                is15SecondsActive = false; // Desactiva el temporizador de 15 segundos
            }
        }
        else if (cont_secs >= c_secIn8hours)
        {
            cont_secs = 0; // Reinicia el contador de segundos
            // Si el temporizador de 8 horas ha terminado, apaga el pin3 y hace toggle en pin2
            digitalWrite(pin3, HIGH); // Apaga el pin3
            //digitalWrite(pin4, !digitalRead(pin4)); // Cambia el estado del pin4 (toggle)

            // Activa el temporizador de 15 segundos
            is15SecondsActive = true;
        }
        timerSeconds.set(1000); // 8 horas en milisegundos
    }
}
