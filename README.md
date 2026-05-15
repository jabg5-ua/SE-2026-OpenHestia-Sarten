# SE-2026-OpenHestia-Sarten

Proyecto PlatformIO para ESP32-C3 en C++ (Arduino) que integra:
- Bluetooth Low Energy (BLE) para publicar telemetría.
- Lectura de acelerómetro por I2C.
- Inferencia TinyML ligera sobre aceleración (probabilidad de caída y actividad).
- Lectura de 4 termistores de 100kΩ por ADC.

## Pines usados

- Termistores: GPIO6, GPIO7, GPIO8 y GPIO9.

## Compilación

```bash
pio run
```

## Monitoreo serie

```bash
pio device monitor
```
