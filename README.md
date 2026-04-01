| | | | | | | | | | | | |  
|-|-|-|-|-|-|-|-|-|-|-|-|  
| **Supported Targets** | **ESP32** | **ESP32-C2** | **ESP32-C3** | **ESP32-C5** | **ESP32-C6** | **ESP32-C61** | **ESP32-H2** | **ESP32-H21** | **ESP32-P4** | **ESP32-S2** | **ESP32-S3** |   
   
**POD_042** ** – ESP32 firmware**  
Central processing unit of the POD-042 system, implemented on an ESP32 DevKit V1.Responsible for BLE telemetry, STM32 communication, and system state management.  
**Hardware Required**  
- A development board ESP32 DevKit V1 and STM32 (see stm32 branch).  
- A push button   
**Pin Configuration**  
- GPIO 16: Uart RX — receives data from STM32.  
- GPIO 17: Uart TX — sends commands to STM32.  
- GPIO 4: Physical button — IMU toggle / make signal  
   
**Dependencies**  
- **ESP-IDF v5.5.1** — Espressif IoT Development Framework.  
- **esp_hid** — HID/GATT BLE component (included in ESP-IDF)  
- **nvs_flash — Non-valatile storage for BLE bonding.  
**Commands sent to STM32**  
| | |  
|-|-|  
| STM:PING | Heartbeat request |   
| STM:BATT | Battery level request |   
| STM:LED | LED toggle request |   
| STM:MPU | IMU enable / disable toggle |   
   
**Menuconfig requirements**  
   
**Build and Flash**  
—TERMINAL—  
#Set up ESP-IDF environment  
**$HOME/esp/esp-idf/export.sh**  
#Build  
**Idf.py build**  
#flash and monitor  
**Idf.py flash monitor**  
**Example Output**  
