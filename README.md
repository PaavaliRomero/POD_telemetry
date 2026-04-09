# POD-042  
   
Embedded system based on two microcontrollers, inspired by the PODs from NieR Automata.  
The POD-042 acts as a tactical support unit, reporting orientation, battery level,  
and system status via BLE telemetry in NieR Automata style.  
   
> ⚠️ Work in progress — active development.  
   
---  
   
## Repository Structure  
   
| Branch | Description |  
|--------|-------------|  
| `stm32` | STM32 firmware — nervous system, sensor and peripheral control |  
| `esp32` | ESP32 firmware — central brain, BLE telemetry and state management |  
   
---  
   
## Hardware  
   
| Component | Description |  
|-----------|-------------|  
| ESP32 DevKit V1 | Central processing unit |  
| STM32F103 | Peripheral and sensor control unit |  
| MPU6050 | Orientation sensor — roll, pitch and gesture detection |  
| OLED SSD1306 128x32 | System HUD display |  
| LiPo Battery | Power source with ADC voltage measurement |  
   
---  
   
## Project Status  
   
| Feature | Status |  
|---------|--------|  
| STM32 ↔ ESP32 UART communication | ✅ Complete |  
| MPU6050 orientation and gesture detection | ✅ Progress |  
| LiPo battery ADC measurement | ✅ Complete |  
| OLED SSD1306 interface | ✅ Progress |  
| Intelligent sleep mode | ✅ Complete |  
| BLE telemetry — NieR Automata style | ✅ Progress |  
| GPS integration | 🔄 Planned |  
| Altimeter BMP280 | 🔄 Planned |  
| Distance sensor VL53L0X | 🔄 Planned |  
   
---  
   
## Inspiration  
   
This project is inspired by the POD support units from **NieR Automata** by Yoko Taro.  
The POD-042 replicates the concept of an autonomous tactical assistant that monitors  
its operator and reports system status in real time.  
