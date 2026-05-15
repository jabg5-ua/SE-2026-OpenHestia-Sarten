# SE-2026-OpenHestia-Sarten

Proyecto PlatformIO para ESP32-C3 en C++ (Arduino) que integra:
- Bluetooth Low Energy (BLE) para publicar telemetría.
- Lectura de acelerómetro por I2C.
- Inferencia TinyML ligera sobre aceleración (probabilidad de caída y actividad).
- Lectura de 4 termistores de 100kΩ usando multiplexor analógico 16 canales.

## Pines usados

- Multiplexor 16 canales:
  - SIG (salida analógica a ADC ESP32-C3): GPIO6.
  - Selectores: S0=GPIO7, S1=GPIO8, S2=GPIO10, S3=GPIO18.
  - Termistores usados: canales 0, 1, 2 y 3 del multiplexor.
- Telemetría térmica:
  - `thermistors_c`: arreglo de los 4 valores individuales.
  - `thermistors_merged_c`: valor unificado (promedio de valores válidos).

## Compilación

```bash
pio run
```

## Monitoreo serie

```bash
pio device monitor
```
