#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <jabg5-ua-project-1_inferencing.h>

// ── VARIABLE GLOBAL PARA LA TEMPERATURA ───────────────────────
volatile float temperaturaMediaGlobal = 0.0f;

// ── CONFIGURACIÓN BLE ─────────────────────────────────────────
BLEAdvertising *pAdvertising;

//direccion de acelerometro
Adafruit_MPU6050 mpu;
const int MPU_ADDR = 0x68;

// Estructura binaria compacta para enviar por el aire
struct __attribute__((packed)) PayloadSarten {
    uint16_t companyId = 0xFFFF; // Identificador propio
    uint8_t sartenColocada;      // 1 = Sí (dejar sartén detectado)
    float temperaturaMedia;      // Sacada de la variable global
};

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
                //Serial.printf("  C%d → raw=%.0f  T= %.2f °C\n", ch, raw, temp);
                suma += temp;
                validos++;
            }
        }

        if (validos > 0)
        {
            float mediaCalculada = suma / validos;
            
            temperaturaMediaGlobal = mediaCalculada;

            Serial.printf("🔥 [TEMP] Media actualizada globalmente: %.2f °C\n", temperaturaMediaGlobal);
        }
        else
            Serial.println("  >> Sin sensores válidos");

        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

void taskInferencia(void *pvParameters) {
    static float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };

    while (true) {
        // 1. Capturar movimiento del MPU6050
        for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += 3) {
            sensors_event_t a, g, temp;
            mpu.getEvent(&a, &g, &temp);

            buffer[ix + 0] = a.acceleration.x;
            buffer[ix + 1] = a.acceleration.y;
            buffer[ix + 2] = a.acceleration.z;

            vTaskDelay(pdMS_TO_TICKS(EI_CLASSIFIER_INTERVAL_MS));
        }

        // 2. Procesar con Edge Impulse
        signal_t signal;
        numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
        
        ei_impulse_result_t result = { 0 };
        EI_IMPULSE_ERROR r = run_classifier(&signal, &result, false);
        
        if (r == EI_IMPULSE_OK) {
            // 3. ENCONTRAR EL ESTADO MÁS ALTO (GANADOR)
            int max_idx = 0;
            float max_val = 0.0f;
            
            for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
                if (result.classification[ix].value > max_val) {
                    max_val = result.classification[ix].value;
                    max_idx = ix;
                }
            }

            // Guardamos el nombre de la etiqueta ganadora
            const char* etiquetaGanadora = result.classification[max_idx].label;
            
            Serial.printf("🤖 [IA] Estado más probable: %s (%.2f%%)\n", etiquetaGanadora, max_val * 100.0f);

            // 4. ¿EL GANADOR ES "dejar sarten"?
            // ⚠️ ATENCIÓN: Asegúrate de que "dejar sarten" esté escrito EXACTAMENTE igual que en Edge Impulse (ej: "dejar_sarten")
            if (strcmp(etiquetaGanadora, "dejar sarten") == 0) {
                
                Serial.println("📡 [BLE] ¡Ventana abierta! Sincronizando con el fogón...");

                // Preparar los datos introduciendo la TEMPERATURA de la variable global
                PayloadSarten datos;
                datos.sartenColocada = 1;
                datos.temperaturaMedia = temperaturaMediaGlobal; // <-- Captura el valor global actual

                // Meter los datos en el paquete publicitario
                std::string dataStr((char*)&datos, sizeof(PayloadSarten));
                BLEAdvertisementData oAdvertisementData;
                oAdvertisementData.setManufacturerData(dataStr);
                pAdvertising->setAdvertisementData(oAdvertisementData);

                // Configurar ráfaga ultra rápida (20 ms de intervalo)
                pAdvertising->setMinInterval(32); 
                pAdvertising->setMaxInterval(32); 
                
                // ENCENDER BLUETOOTH
                pAdvertising->start(); 

                // ── VENTANA DE TIEMPO DE SINCRONIZACIÓN ──────────────────
                // Mantenemos el Bluetooth emitiendo durante 4 segundos libres
                vTaskDelay(pdMS_TO_TICKS(4000)); 
                // ─────────────────────────────────────────────────────────

                // APAGAR BLUETOOTH
                pAdvertising->stop();
                Serial.println("🛑 [BLE] Ventana de tiempo cerrada. Bluetooth apagado.");
            }
        }
        
        // Pausa de cortesía de medio segundo antes de volver a evaluar el movimiento
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

    // INICIALIZAR BLE (Se configura pero se queda en reposo sin transmitir)
    BLEDevice::init("Sarten-C3");
    pAdvertising = BLEDevice::getAdvertising();

    xTaskCreate(taskTemperatura, "TareaTemp", 4096, NULL, 1, NULL);
    xTaskCreate(taskInferencia, "TareaIA", 16384, NULL, 1, NULL);

}

void loop() {}