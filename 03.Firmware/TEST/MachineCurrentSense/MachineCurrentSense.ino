#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include <array>
#include <cmath>
#include "Adafruit_SSD1306.h"
#include <pico/multicore.h>
#include "Arduino.h"

// #define LABVIEW_1
//     #define LABVIEW_2
#define PUERTO_SERIE

// Display Constanst
const uint16_t C_PIN_MISO = 16;    // MISO Display
const uint16_t C_PIN_SS = 17;      // SS Display
const uint16_t C_PIN_SCK = 18;     // SCK Display
const uint16_t C_PIN_MOSI = 19;    // MOSI Display
const uint16_t C_PIN_DSP_RST = 20; // Reset Display

Adafruit_SSD1306 display(128, 64, C_PIN_MOSI, C_PIN_SCK, C_PIN_MISO, C_PIN_DSP_RST, C_PIN_SS);

bool update_display = false;
// Configuración
const int NUM_MUESTRAS = 1000;          // Número total de muestras
const int NUM_MUESTRAS_ADQ = 1500;      // Número total de muestras
uint16_t datosADC[NUM_MUESTRAS_ADQ];    // Arreglo para datos originales del ADC
float datos2Filt[NUM_MUESTRAS];         // Arreglo para datos originales del ADC
uint16_t datosADC2[NUM_MUESTRAS];       // Arreglo para datos originales del ADC
float datosFiltrados[NUM_MUESTRAS_ADQ]; // Arreglo para datos filtrados
volatile bool dma_completado = false;   // Bandera para indicar finalización del DMA

// Pines PWM
const uint PWM_PIN_1 = 2; // Primer pin de PWM
const uint PWM_PIN_2 = 3; // Segundo pin de PWM

// Constantes para el rango de frecuencia de entrada
int frecuencia_min = 0;   // Frecuencia mínima (Hz)
int frecuencia_max = 150; // Frecuencia máxima (Hz)
int duty_min = 0;         // Frecuencia mínima (Hz)
int duty_max = 100;       // Frecuencia máxima (Hz)

// Constante para la frecuencia del PWM
const int PWM_FREQUENCY = 50000; // Frecuencia del PWM (50 kHz)

// Variables de configuración PWM
uint slice_num_1, slice_num_2;
uint channel_1, channel_2;

// Calculo Valor maximo y umbral
const int UMBRAL_MIN = 0;    // Umbral minimo (cuentas)
const int UMBRAL_MAX = 4095; // Umbral máximo (cuentas)

const uint NUM_MAX_COUNT = 10;
const uint PERCENT_THRESHOLD_COIL = 25;
const uint PERCENT_THRESHOLD_DK = 90;
uint percent_threshold = PERCENT_THRESHOLD_COIL;

uint max_values[NUM_MAX_COUNT];
uint actual_max_1 = 0;
uint actual_max_2 = 0;
uint min_values[NUM_MAX_COUNT];
uint actual_min_1 = 0;
uint actual_min_2 = 0;
uint umbral_top;
uint umbral_bot;
uint umbral;

constexpr int MAX_CICLOS = 100;         // Número máximo de ciclos que esperas detectar
constexpr int MAX_TIEMPOS_CICLOS = 100; // Número máximo de tiempos entre ciclos

// Tracking and DK modes

const int DK_MODE_PIN = 8;
const int TRACK_MODE_PIN = 6;

bool prev_dk_mode_pin = false;
bool actual_dk_mode_pin = false;
bool prev_track_mode_pin = false;
bool actual_track_mode_pin = false;

bool dk_mode = false;
bool track_mode = false;

// Media de frecuencia y duty
int cont_zero = 0;
const int NUM_BUFF = 4;

float dutyCyclePromedio = 0.0;
float dutyCyclePromedio_sample = 0.0;
float dutyCyclePromedio_prev = 50.0;
float buffer_duty[NUM_BUFF];

float frecuencia = 0.0;
float frecuencia_sample = 0.0;
float frecuencia_prev = 100.0;
float buffer_frec[NUM_BUFF];

float dutyCycles[MAX_CICLOS];               // Arreglo para almacenar los duty cycles de cada ciclo
int tiemposEntreCiclos[MAX_TIEMPOS_CICLOS]; // Arreglo para almacenar los tiempos entre ciclos
int numCiclosDetectados = 0;                // Contador de ciclos detectados
int numTiemposDetectados = 0;               // Contador de tiempos entre ciclos detectados

// Filter order 4, F=200Hz
float gain_200 = 75223.438632;
float c1_200 = 3.6717290892;
float c2_200 = 5.0679983867;
float c3_200 = 3.1159669252;
float c4_200 = 0.7199103273;

// Filter order 4, F=300Hz
float gain_300 = 16028.984623;
float c1_300 = 3.5077862074;
float c2_300 = 4.6409024127;
float c3_300 = 2.7426528211;
float c4_300 = 0.6105348076;
// Filter coeficient
float gain = gain_300;
float c1 = c1_300;
float c2 = c2_300;
float c3 = c3_300;
float c4 = c4_300;

// Estado del filtro (historial de entradas y salidas)
float x_hist[5] = {0}; // Historial de entradas
float y_hist[5] = {0}; // Historial de salidas

// Función para aplicar el filtro Butterworth
float aplicarFiltroButterworth(float nuevaEntrada)
{
    // Desplazar las entradas anteriores
    for (int i = 4; i > 0; i--)
    {
        x_hist[i] = x_hist[i - 1];
    }
    x_hist[0] = nuevaEntrada / gain; // Escalar la nueva entrada

    // Desplazar las salidas anteriores
    for (int i = 4; i > 0; i--)
    {
        y_hist[i] = y_hist[i - 1];
    }

    // Calcular la nueva salida utilizando la ecuación del filtro
    y_hist[0] = (x_hist[0] + x_hist[4])       // Coeficientes del numerador (b0, b4)
                + 4 * (x_hist[1] + x_hist[3]) // Coeficientes del numerador (b1, b3)
                + 6 * x_hist[2]               // Coeficiente del numerador (b2)
                + (c1 * y_hist[1])            // Coeficiente del denominador (a1)
                - (c2 * y_hist[2])            // Coeficiente del denominador (a2)
                + (c3 * y_hist[3])            // Coeficiente del denominador (a3)
                - (c4 * y_hist[4]);           // Coeficiente del denominador (a4)

    // Devolver la salida filtrada
    return y_hist[0];
}
int t0;
int dma_channel;
void setup()
{
    multicore_launch_core1(core1_entry);
    Serial.begin(115200);
    display.begin();
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.display();
    delay(1000);

    analogReadResolution(12);
    // Inicializar el ADC

    adc_init();
    adc_gpio_init(26);   // Configurar GPIO 26 (ADC0) como entrada
    adc_select_input(0); // Seleccionar canal ADC0
    adc_set_clkdiv(4800);
    // Configuración del FIFO
    adc_fifo_setup(
        true,  // Activar el FIFO
        true,  // Habilitar DMA/IRQ
        1,     // Nivel del FIFO antes de interrumpir
        false, // No cambiar el bit de signo
        false  // Shift de 12 bits a 16 bits
    );

    // Configuración del DMA
    dma_channel = dma_claim_unused_channel(true); // Reclamar un canal DMA
    dma_channel_config dma_config = dma_channel_get_default_config(dma_channel);
    channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_16); // Transferir 16 bits (2 bytes)
    channel_config_set_read_increment(&dma_config, false);           // No incrementar dirección de lectura
    channel_config_set_write_increment(&dma_config, true);           // Incrementar dirección de escritura
    channel_config_set_dreq(&dma_config, DREQ_ADC);                  // Usar señal del ADC como disparador

    // Configurar la transferencia DMA
    dma_channel_configure(
        dma_channel,      // Canal DMA
        &dma_config,      // Configuración
        datosADC,         // Dirección de escritura (arreglo)
        &adc_hw->fifo,    // Dirección de lectura (FIFO del ADC)
        NUM_MUESTRAS_ADQ, // Número de transferencias
        false             // No iniciar aún
    );

    // Configurar interrupción DMA
    dma_channel_set_irq0_enabled(dma_channel, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    // Iniciar el ADC en modo continuo
    t0 = micros();
    adc_run(true);
    dma_channel_start(dma_channel); // Iniciar el DMA

    // Configuración del pin PWM 1
    gpio_set_function(PWM_PIN_1, GPIO_FUNC_PWM);
    slice_num_1 = pwm_gpio_to_slice_num(PWM_PIN_1);
    channel_1 = pwm_gpio_to_channel(PWM_PIN_1);

    // Configurar divisor para 50 kHz
    uint32_t clock_div = 125000000 / PWM_FREQUENCY;
    pwm_set_clkdiv(slice_num_1, (float)clock_div / 65536.0); // Ajustar divisor
    pwm_set_wrap(slice_num_1, 255);                          // Resolución de 8 bits
    pwm_set_enabled(slice_num_1, true);                      // Habilitar PWM

    // Configuración del pin PWM 2
    gpio_set_function(PWM_PIN_2, GPIO_FUNC_PWM);
    slice_num_2 = pwm_gpio_to_slice_num(PWM_PIN_2);
    channel_2 = pwm_gpio_to_channel(PWM_PIN_2);

    // Configurar divisor para 50 kHz
    pwm_set_clkdiv(slice_num_2, (float)clock_div / 65536.0);
    pwm_set_wrap(slice_num_2, 255);
    pwm_set_enabled(slice_num_2, true);

    Serial.println("Iniciando captura de datos...");
}
unsigned long tiempoUltimaCaptura = 0;
const unsigned long intervaloCaptura = 200;
int j = 0;
void loop()
{
    while (j < 20)
    {
        unsigned long tiempoActual = millis();
        if (dma_completado)
        {
            // j++;
            //  Serial.printf("Tiempos:%d", micros() - t0);
            //  Serial.println("Captura completada. Filtrando datos...");
            //   Enviar datos filtrados al puerto serie
            for (int i = 0; i < NUM_MUESTRAS_ADQ; i++)
            {
                // Serial.println(datosADC[i]);
            }
            // Sensado de Datos por A1
            // for (int i = 0; i < NUM_MUESTRAS; i++)
            // {
            //     datosADC2[i] = analogRead(A1);
            //     delayMicroseconds(100);
            // }
            // for (int i = 0; i < NUM_MUESTRAS; i++)
            // {
            //     Serial.println(datosADC2[i]);
            // }

            // Filtrar cada muestra y almacenarla en el arreglo de datos filtrados
            t0 = micros();
            actual_max_1 = UMBRAL_MIN;
            actual_max_2 = UMBRAL_MIN;
            actual_min_1 = UMBRAL_MAX;
            actual_min_2 = UMBRAL_MAX;
            for (int i = 0; i < NUM_MUESTRAS_ADQ; i++)
            {
                float entrada = (float)datosADC[i];                    // Convertir a flotante
                datosFiltrados[i] = aplicarFiltroButterworth(entrada); // Filtrar y almacenar
                if (i < (NUM_MUESTRAS_ADQ / 2))
                {
                    if (datosFiltrados[i] > actual_max_1)
                    {
                        actual_max_1 = datosFiltrados[i];
                    }
                    if (datosFiltrados[i] < actual_min_1)
                    {
                        actual_min_1 = datosFiltrados[i];
                    }
                }
                else
                {
                    if (datosFiltrados[i] > actual_max_2)
                    {
                        actual_max_2 = datosFiltrados[i];
                    }
                    if (datosFiltrados[i] < actual_min_2)
                    {
                        actual_min_2 = datosFiltrados[i];
                    }
                }
            }
            umbral_top = 0;

            for (size_t i = NUM_MAX_COUNT - 1; i > 1; i--)
            {
                max_values[i] = max_values[i - 1];
                umbral_top += max_values[i];
            }
            max_values[0] = actual_max_1;
            max_values[1] = actual_max_2;

            umbral_top = (umbral_top + actual_max_1 + actual_max_2) / NUM_MAX_COUNT;

            umbral_bot = 0;
            for (size_t i = NUM_MAX_COUNT - 1; i > 1; i--)
            {
                min_values[i] = min_values[i - 1];
                umbral_bot += min_values[i];
            }
            min_values[0] = actual_min_1;
            min_values[1] = actual_min_2;
            umbral_bot = (umbral_bot + actual_min_1 + actual_min_2) / NUM_MAX_COUNT;

            umbral = umbral_bot + (umbral_top - umbral_bot) * percent_threshold / 100;

            umbral = constrain(umbral, UMBRAL_MIN, UMBRAL_MAX);

            // Serial.printf("Tiempos:%d\n", micros() - t0);

            for (int i = 0; i < NUM_MUESTRAS; i++)
            {
                datos2Filt[i] = datosFiltrados[499 + i];
            }
            // Calcular el duty cycle y la frecuencia usando un umbral de 0.3 V
            calcularDutyCycleYFrecuencia(datos2Filt, NUM_MUESTRAS, umbral, 10000.0);

            // Serial.println("Datos filtrados almacenados. Enviando al puerto serie...");

            // Enviar datos filtrados al puerto serie
            for (int i = 0; i < NUM_MUESTRAS_ADQ; i++)
            {
                // Serial.printf(datosFiltrados[i]);

#ifdef LABVIEW_1
                Serial.printf("%d-%.2f\n", (int)datosADC[i], datosFiltrados[i]);
#endif
            }
#ifdef LABVIEW_1
            Serial.printf("!");
#endif
            // Serial.println("Datos filtrados procesados.");

            // Reiniciar para la próxima captura
            dma_completado = false;

            // Reconfigurar DMA y reiniciar ADC
            // dma_channel_start(dma_channel); // Iniciar el DMA nuevamente
            //

            reiniciarDMA();

            t0 = micros();
        }

        // Evitar ejecutar capturas antes de que pase el intervalo
        if (tiempoActual - tiempoUltimaCaptura >= intervaloCaptura)
        {
            tiempoUltimaCaptura = tiempoActual;
            // Serial.println("Nueva captura...");
            adc_run(true);
            dma_channel_start(dma_channel);
        }
    }
    while (1)
    {
        /* code */
    }
}
void core1_entry()
{
    while (true)
    {
        prev_track_mode_pin = actual_track_mode_pin;
        actual_track_mode_pin = digitalRead(TRACK_MODE_PIN);

        if ((actual_track_mode_pin == false) && (prev_track_mode_pin == true))
        {
            if (track_mode == false)
            {
                track_mode = true;
                frecuencia_min = frecuencia - 20;
                frecuencia_max = frecuencia + 20;
                duty_min = dutyCyclePromedio - 20;
                duty_max = dutyCyclePromedio + 20;
            }
            else
            {
                track_mode = false;
                frecuencia_min = 0;
                frecuencia_max = 150;
                duty_min = 0;
                duty_max = 100;
            }
        }

        if (digitalRead(DK_MODE_PIN))
        {
            // DK MODE
            percent_threshold = PERCENT_THRESHOLD_DK;
            gain = gain_200;
            c1 = c1_200;
            c2 = c2_200;
            c3 = c3_200;
            c4 = c4_200;
        }
        else
        {
            // COIL MODE
            percent_threshold = PERCENT_THRESHOLD_COIL;
            gain = gain_300;
            c1 = c1_300;
            c2 = c2_300;
            c3 = c3_300;
            c4 = c4_300;
        }

        if (update_display == true)
        {
            display.clearDisplay();
            display.setTextSize(2);
            display.setTextColor(WHITE);
            display.setCursor(0, 0);
            display.print("U:");
            display.setCursor(30, 0);
            display.print(umbral);
            display.setCursor(0, 25);
            display.print("F: ");
            display.setCursor(0, 50);
            display.print("D: ");
            display.setCursor(30, 25);
            display.print(frecuencia);
            display.setCursor(30, 50);
            display.print(dutyCyclePromedio);
            display.display();

            update_display = false;
#ifdef PUERTO_SERIE
            // Imprimir los valores configurados
            Serial.printf("Tracking: %d, Umbral: %d, Frecuencia: %d Hz, Cont: %d, DutyMedido: %d,\n", track_mode, umbral, (int)frecuencia_sample, cont_zero, (int)dutyCyclePromedio);
           // Serial.printf("Freq_prev: %d, Freq_sample: %d, Dut_prev: %d, Dut_Sample: %d,\n", (int)frecuencia_prev, (int)frecuencia_sample, (int)dutyCyclePromedio_prev, (int)dutyCyclePromedio_sample);
#endif
        }
        delay(50);
    }
}
// Manejador de interrupción del DMA
void dma_irq_handler()
{
    // Marcar como completado y detener el ADC
    dma_completado = true;
    adc_run(false);
    dma_hw->ints0 = 1u << dma_channel; // Limpiar la bandera de interrupción
}

void calcularDutyCycleYFrecuencia(const float *datosFiltrados, int numMuestras, uint32_t umbral, float frecuenciaMuestreo)
{
    int muestrasPorEncima = 0;
    int muestrasPorCiclo = 0;
    numCiclosDetectados = 0;
    numTiemposDetectados = 0;
    bool dentroDelCiclo = false;

    // Variables para el cálculo de frecuencia
    int tiempoAnterior = 0;
    // Serial.printf("umbral:%d\n", umbral);
    for (int i = 1; i < numMuestras; i++)
    {
        // Detectar cruce por el umbral (de bajada o subida)
        bool cruceHaciaArriba = datosFiltrados[i - 1] < umbral && datosFiltrados[i] >= umbral;
        bool cruceHaciaAbajo = datosFiltrados[i - 1] >= umbral && datosFiltrados[i] < umbral;

        // Iniciar un nuevo ciclo
        if (cruceHaciaArriba)
        {
            // Serial.printf("Subida!\n");
            if (muestrasPorCiclo > 0)
            {
                // Serial.printf("Nuevo Ciclo!\n");
                //  Finalizamos el ciclo anterior y calculamos el duty cycle
                if (numCiclosDetectados < MAX_CICLOS)
                {
                    // Serial.printf("%d-%d\n", (int)muestrasPorEncima, (int)muestrasPorCiclo);
                    float dutyCycle = (float)muestrasPorEncima / muestrasPorCiclo * 100.0;
                    dutyCycles[numCiclosDetectados++] = dutyCycle;
                }

                // Guardar el tiempo entre ciclos si ya hemos detectado al menos dos ciclos
                if (numCiclosDetectados > 1)
                {
                    int tiempoEntreCiclo = i - tiempoAnterior; // Tiempo entre el ciclo actual y el anterior (en muestras)
                    if (numTiemposDetectados < MAX_TIEMPOS_CICLOS)
                    {
                        tiemposEntreCiclos[numTiemposDetectados++] = tiempoEntreCiclo;
                    }
                }
            }

            // Reiniciar contadores para el nuevo ciclo
            dentroDelCiclo = true;
            muestrasPorEncima = 0;
            muestrasPorCiclo = 0;
            tiempoAnterior = i;
        }

        // Contar muestras por encima del umbral dentro del ciclo

        if (datosFiltrados[i] > umbral)
        {
            muestrasPorEncima++;
        }
        muestrasPorCiclo++;
    }

    // Calcular el promedio de los duty cycles
    dutyCyclePromedio = 0.0;
    if (numCiclosDetectados > 0)
    {
        for (int i = 0; i < numCiclosDetectados - 1; i++) // l apriera muestra del calculo del Duty es erronea por lo que no la usamos para el calculo
        {
            dutyCyclePromedio += dutyCycles[1 + i];
        }
        dutyCyclePromedio /= (numCiclosDetectados - 1);
        dutyCyclePromedio_sample = constrain(dutyCyclePromedio, 0, 100);
    }

    // Calcular la frecuencia si tenemos tiempos entre ciclos
    frecuencia_sample = 0.0;
    if (numTiemposDetectados > 1)
    {
        // Calcular el promedio de los tiempos entre ciclos
        float tiempoTotal = 0.0;
        for (int i = 0; i < numTiemposDetectados; i++)
        {
            tiempoTotal += tiemposEntreCiclos[i];
        }
        float tiempoPromedioEntreCiclos = tiempoTotal / numTiemposDetectados;
        // Convertir el tiempo promedio a segundos (usando la frecuencia de muestreo)
        float tiempoPromedioSegundos = tiempoPromedioEntreCiclos / frecuenciaMuestreo;
        // Calcular la frecuencia (inverso del tiempo)
        frecuencia_sample = 1.0 / tiempoPromedioSegundos;
    }

#ifdef LABVIEW_2
    Serial.printf("%d-%d\n", (int)frecuencia, (int)dutyCyclePromedio);
#endif
    frecuencia_prev = 0;
    dutyCyclePromedio_prev = 0;
    for (int i = NUM_BUFF - 1; i >= 0; i--)
    {
        dutyCyclePromedio_prev += buffer_duty[i];
        frecuencia_prev += buffer_frec[i];
        if (i > 0)
        {
            buffer_duty[i] = buffer_duty[i - 1];
            buffer_frec[i] = buffer_frec[i - 1];
        }
    }
    frecuencia_prev = frecuencia_prev / NUM_BUFF;
    dutyCyclePromedio_prev = dutyCyclePromedio_prev / NUM_BUFF;

    if ((frecuencia_sample <= (frecuencia_prev - 30)) || (frecuencia_sample >= (frecuencia_prev  + 30)))
    {
    }
    else
    {
        frecuencia = frecuencia_sample;
    }
    if ((dutyCyclePromedio_sample <= (dutyCyclePromedio_prev - 30)) || (dutyCyclePromedio_sample >= (dutyCyclePromedio_prev + 30)))
    {
    }
    else
    {
        dutyCyclePromedio = dutyCyclePromedio_sample;
    }

    buffer_duty[0] = (dutyCyclePromedio_prev + dutyCyclePromedio_sample) / 2;
    buffer_frec[0] = (frecuencia_prev + frecuencia_sample) / 2;

    if (umbral < 100)
    {
        frecuencia = 0;
        dutyCyclePromedio = 0;
        actualizarPWM(frecuencia, dutyCyclePromedio);
        cont_zero = 0;
    }
    if ((frecuencia <= 0) || (dutyCyclePromedio <= 0))
    {
        if (cont_zero++ == 5)
        {
            actualizarPWM(frecuencia, dutyCyclePromedio);
            cont_zero = 0;
        }
    }
    else
    {
        actualizarPWM(frecuencia, dutyCyclePromedio);
        cont_zero = 0;
    }
    update_display = true;

    // Serial.printf("---------------------------\n");
}
void reiniciarDMA()
{
    // Detener el DMA si está activo
    dma_channel_abort(dma_channel);

    // Configurar el canal nuevamente
    dma_channel_config dma_config = dma_channel_get_default_config(dma_channel);
    channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_16); // Tamaño de transferencia de 16 bits
    channel_config_set_read_increment(&dma_config, false);           // No incrementar lectura
    channel_config_set_write_increment(&dma_config, true);           // Incrementar escritura
    channel_config_set_dreq(&dma_config, DREQ_ADC);                  // Disparador: ADC FIFO

    dma_channel_configure(
        dma_channel,      // Canal DMA
        &dma_config,      // Configuración
        datosADC,         // Buffer de destino
        &adc_hw->fifo,    // Dirección de lectura
        NUM_MUESTRAS_ADQ, // Número de transferencias
        false             // No iniciar aún
    );

    // Limpiar cualquier bandera pendiente del DMA
    dma_hw->ints0 = 1u << dma_channel;
}

void actualizarPWM(int frecuenciaMedida, float dutyMedido)
{
    float dutyPWM1;
    float dutyPWM2;

    // Calcular duty cycle para el PWM1 basado en la frecuencia
    // Frecuencia
    dutyPWM1 = (frecuenciaMedida - frecuencia_min) * 100.0 / (frecuencia_max - frecuencia_min);
    // DUTY
    dutyPWM2 = (dutyMedido - duty_min) * 100.0 / (duty_max - duty_min);

    //// Umbral
    // dutyPWM1 = (umbral - UMBRAL_MIN) * 100.0 / (UMBRAL_MAX - UMBRAL_MIN);

    // Configurar PWM1
    uint16_t levelPWM1 = (uint16_t)(dutyPWM1 * 255.0 / 100.0);
    pwm_set_chan_level(slice_num_1, channel_1, levelPWM1);

    // Configurar PWM2
    uint16_t levelPWM2 = (uint16_t)(dutyPWM2 * 255.0 / 100.0);
    pwm_set_chan_level(slice_num_2, channel_2, levelPWM2);
}