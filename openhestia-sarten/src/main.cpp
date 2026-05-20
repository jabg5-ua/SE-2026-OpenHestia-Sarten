#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <jabg5-ua-project-1_inferencing.h>

//direccion de acelerometro
Adafruit_MPU6050 mpu;
const int MPU_ADDR = 0x68;

// ── Pines mux ─────────────────────────────────────────────────
#define MUX_S0 1
#define MUX_S1 2
#define MUX_SIG 0

// ── Parámetros NTC ────────────────────────────────────────────
#define SERIESRESISTOR 100000
#define NOMINAL_RESISTANCE 100000
#define NOMINAL_TEMPERATURE 25
#define BCOEFFICIENT 3950

#define SAMPLE_INTERVAL_MS 7000

// ─────────────────────────────────────────────────────────────

void selectMuxChannel(uint8_t ch)
{
    digitalWrite(MUX_S0, (ch >> 0) & 1);
    digitalWrite(MUX_S1, (ch >> 1) & 1);
    vTaskDelay(pdMS_TO_TICKS(100));
}

float adcToTemperature(float ADCvalue)
{
    if (ADCvalue <= 0 || ADCvalue >= 4095)
        return NAN;

    float Resistance = (4095.0 / ADCvalue) - 1.0;
    Resistance = SERIESRESISTOR / Resistance;

    float steinhart = Resistance / NOMINAL_RESISTANCE;
    steinhart = log(steinhart);
    steinhart /= BCOEFFICIENT;
    steinhart += 1.0 / (NOMINAL_TEMPERATURE + 273.15);
    steinhart = 1.0 / steinhart;
    steinhart -= 273.15;

    return steinhart;
}

void taskTemperatura(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(SAMPLE_INTERVAL_MS);

    while (true)
    {
        Serial.println("─────────────────────────────");

        float suma = 0.0f;
        uint8_t validos = 0;

        for (uint8_t ch = 0; ch < 3; ch++)
        {
            selectMuxChannel(ch);

            analogRead(MUX_SIG);
            vTaskDelay(pdMS_TO_TICKS(100));

            int acum = 0;
            for (uint8_t s = 0; s < 32; s++)
                acum += analogRead(MUX_SIG);
            float raw = acum / 32.0f;

            float temp = adcToTemperature(raw);

            if (isnan(temp))
            {
                Serial.printf("  C%d → sin sensor (raw=%.0f)\n", ch, raw);
            }
            else
            {
                Serial.printf("  C%d → raw=%.0f  T= %.2f °C\n", ch, raw, temp);
                suma += temp;
                validos++;
            }
        }

        if (validos > 0)
            Serial.printf("  >> MEDIA: %.2f °C (%d sensor/es)\n", suma / validos, validos);
        else
            Serial.println("  >> Sin sensores válidos");

        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

void taskInferencia(void *pvParameters) {
    while (true) {
        // 1. Crear el buffer para la IA
        float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };

        // 2. Llenar el buffer a la frecuencia exacta que pide Edge Impulse
        for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += 3) {
            uint64_t next_tick = micros() + (EI_CLASSIFIER_INTERVAL_MS * 1000);
            
            sensors_event_t a, g, temp;
            mpu.getEvent(&a, &g, &temp);

            buffer[ix + 0] = a.acceleration.x;
            buffer[ix + 1] = a.acceleration.y;
            buffer[ix + 2] = a.acceleration.z;

            // Esperar al siguiente tick, pero cediendo tiempo al ESP32 (FreeRTOS)
            while (micros() < next_tick) { 
                taskYIELD(); 
            }
        }

        // 3. Convertir buffer y ejecutar clasificador
        signal_t signal;
        numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
        
        ei_impulse_result_t result = { 0 };
        EI_IMPULSE_ERROR r = run_classifier(&signal, &result, false);
        
        if (r == EI_IMPULSE_OK) {
            Serial.println("=== PREDICCIÓN SARTÉN ===");
            for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
                Serial.printf("  %s: %.2f\n", result.classification[ix].label, result.classification[ix].value);
            }
        }
        
        // Pausa entre inferencias para no saturar
        vTaskDelay(pdMS_TO_TICKS(500)); 
    }
}

void setup()
{
    Serial.begin(115200);

    pinMode(MUX_S0, OUTPUT);
    pinMode(MUX_S1, OUTPUT);
    pinMode(MUX_SIG, INPUT);

    analogReadResolution(12);

    xTaskCreate(taskTemperatura, "TareaTemp", 4096, NULL, 1, NULL);

    // Iniciar bus I2C
    Wire.begin(8, 9);

    // Despertar manual del MPU6050
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B); 
    Wire.write(0);    
    Wire.endTransmission(true);
    vTaskDelay(pdMS_TO_TICKS(100)); // En FreeRTOS usamos vTaskDelay en lugar de delay()

    if (mpu.begin(MPU_ADDR, &Wire, 0)) {
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    } else {
        Serial.println("¡Error al iniciar el MPU6050!");
    }

    xTaskCreate(taskInferencia, "TareaIA", 16384, NULL, 1, NULL);

}

void loop() {}