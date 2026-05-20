#include <Arduino.h>
#include <math.h>

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

void setup()
{
    Serial.begin(115200);

    pinMode(MUX_S0, OUTPUT);
    pinMode(MUX_S1, OUTPUT);
    pinMode(MUX_SIG, INPUT);

    analogReadResolution(12);

    xTaskCreate(taskTemperatura, "TareaTemp", 4096, NULL, 1, NULL);
}

void loop() {}