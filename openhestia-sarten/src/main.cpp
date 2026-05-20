#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// 1. Incluyes la librería que te generó Edge Impulse
#include <jabg5-ua-project-1_inferencing.h> 

Adafruit_MPU6050 mpu;

const int MPU_ADDR = 0x68; 

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    // 1. Iniciar bus I2C explícitamente en los pines del Lolin32
    Wire.begin(21, 22);

    Serial.println("Aplicando despertar manual al MPU6050 (Registro 0x6B)...");
    
    // --- TU TRUCO MANUAL PARA FORZAR EL DESPERTAR ---
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B); // Registro de energía (PWR_MGMT_1)
    Wire.write(0);    // Escribir 0 para despertar
    Wire.endTransmission(true);
    
    delay(100); // Darle tiempo al chip para que se estabilice
    // ------------------------------------------------

    // 2. Ahora que está despierto, intentamos que Adafruit lo reconozca
    if (!mpu.begin(MPU_ADDR, &Wire, 0)) {
        Serial.println("¡Adafruit sigue quejándose! (Posible chip clon remarcado)");
        // Si sale este mensaje, el chip no es un MPU6050 original, 
        // pero igual podemos leerlo a lo bruto saltándonos Adafruit.
    } else {
        Serial.println("MPU6050 Conectado exitosamente con la librería.");
        
        // Configuración de rangos (solo si Adafruit lo aceptó)
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    }
}

void loop() {
    // 3. Crear el buffer donde Edge Impulse espera recibir los datos
    // El tamaño debe ser exactamente EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE
    float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };

    // 4. Llenar el buffer leyendo el sensor de forma consecutiva
    for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += 3) {
        unsigned long long next_tick = micros() + (EI_CLASSIFIER_INTERVAL_MS * 1000);
        
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        // Insertamos X, Y, Z del acelerómetro (o giroscopio, según entrenaste tu modelo)
        buffer[ix + 0] = a.acceleration.x;
        buffer[ix + 1] = a.acceleration.y;
        buffer[ix + 2] = a.acceleration.z;

        // Mantener la frecuencia de muestreo exacta que pide el modelo
        while (micros() < next_tick) { /* esperar */ }
    }

    // 5. Convertir el buffer al formato interno de Edge Impulse
    signal_t signal;
    int err = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
    if (err != 0) {
        Serial.printf("Error al crear la señal desde el buffer (%d)\n", err);
        return;
    }

    // 6. Ejecutar la inferencia (Predicción)
    ei_impulse_result_t result = { 0 };
    EI_IMPULSE_ERROR r = run_classifier(&signal, &result, false);
    if (r != EI_IMPULSE_OK) {
        Serial.printf("Error al ejecutar el clasificador (%d)\n", r);
        return;
    }

    // 7. Imprimir los resultados en el Monitor Serie
    Serial.printf("Predicciones:\n");
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        Serial.printf("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
    }
    
    //delay(300); // Esperar un segundo antes de la siguiente predicción
}