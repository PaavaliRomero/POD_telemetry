# POD-042

Sistema embebido de dos microcontroladores inspirado en los PODs de NieR Automata.

## Estructura del repositorio

| Rama | Contenido |
|------|-----------|
| `stm32` | Código del STM32 — sistema nervioso del POD |
| `esp32` | Código del ESP32 — cerebro del POD |

## Hardware

- ESP32 DevKit
- STM32F103
- MPU6050 — sensor de orientación
- OLED SSD1306 128x32
- Batería LiPo con medición ADC

## Arquitectura
```
STM32 ←── UART ──→ ESP32 ←── BLE ──→ Celular
```

## Estado del proyecto

Trabajo en progreso -- ⚠️ Work in progress — active development.
