**POD-042**  
Embedded system based on two microcontrollers, inspired by the PODs from NieR Automata.The POD-042 acts as a tactical support unit, reporting orientation, battery level,and system status via BLE telemetry in NieR Automata style.  
**Repository structure**  
| | |  
|-|-|  
| **Branch** | **Description** |   
| stm32 | STM32 firmware — nervous system, sensor and peripheral control |   
| esp32 | ESP32 firmware — central brain, BLE telemetry and state management |   
   
**Hardware**  
- ESP32 DevKit  
- STM32F103  
- MPU6050 — Orientation sensor — roll, pitch and gesture detection  
- OLED SSD1306 128x32  
- Batería LiPo con medición ADC  
**Ar** **chitecture**  
STM32 ←→ UART ←→ ESP32 ←→ BLE ←→ Celular  
   
   
**Proyect status**  
| | |  
|-|-|  
| Feature | Status |   
| STM32 ↔ ESP32 UART communication | ✅ Complete |   
| MPU6050 orientation and gesture detection | ✅ Complete |   
| LiPo battery ADC measurement | ✅ Complete |   
| OLED SSD1306 interface | Progress |   
| Intelligent sleep mode | ✅ Complete |   
| BLE telemetry — NieR Automata style | Progress |   
| GPS integration | Planned |   
| Altimeter BMP280 | Planned |   
| Distance sensor VL53L0X | Planned |   
   
Trabajo en progreso -- ⚠️ Work in progress — active development.  
