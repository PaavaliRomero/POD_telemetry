# POD-042 — STM32 Firmware  
   
Peripheral control unit of the POD-042 system, implemented on a STM32F103.  
Responsible for sensor reading, peripheral control, sleep management,  
and UART communication with the ESP32.  
   
---  
   
## Hardware Requirements  
   
| Component | Description |  
|-----------|-------------|  
| STM32F103 | Peripheral control microcontroller |  
| MPU6050 | Orientation sensor — roll, pitch and gesture detection |  
| OLED SSD1306 128x32 | System HUD display |  
| LiPo Battery | Power source with ADC voltage measurement |  
| ESP32 (see esp32 branch) | Central processing unit |  
   
### Pin Configuration  
   
| Pin | Function |  
|-----|----------|  
| PA9  | UART TX — sends data to ESP32 |  
| PA10 | UART RX — receives commands from ESP32 |  
| PA3  | Wake button — exits sleep mode via EXTI interrupt |  
| PC13 | LED — status indicator |  
| PB10 | I2C SCL — MPU6050 and OLED clock |  
| PB11 | I2C SDA — MPU6050 and OLED data |  
   
---  
   
## Dependencies  
   
- **STM32CubeIDE** — Development environment  
- **STM32 HAL** — Hardware abstraction layer  
- **Custom MPU6050 driver** — Orientation sensor library  
- **SSD1306 driver** — OLED display library  
   
---  
   
## FSM States  
| State | Description |  
|-------|-------------|  
| `POD_STATE_INIT` | System initialization, MPU6050 config, first battery read |  
| `POD_STATE_IDLE` | Waiting for ESP32 commands, sleep timeout monitoring |  
| `POD_STATE_ERROR` | Unknown command received, returns to IDLE on next command |  
   
---  
   
## Sleep Mode  
   
The STM32 enters STOP mode when:  
- IMU is inactive  
- No activity detected for **60 seconds**  
   
Wake up is triggered by a physical button press on **PA3** via EXTI interrupt.  
   
On wake up the system:  
1. Restores system clock  
2. Reconfigures MPU6050  
3. Reactivates OLED display  
4. Notifies ESP32 with `STM:WAKE`  
   
---  
   
## ESP32 Communication Protocol  
   
UART1 at 115200 baud, 8N1.  
   
### Commands received from ESP32  
   
| Command | Description |  
|---------|-------------|  
| `STM:PING` | Heartbeat request — responds with `ESP:PING:RECEIVE` |  
| `STM:BATT` | Battery level request |  
| `STM:LED` | LED toggle request |  
| `STM:MPU` | IMU enable/disable toggle |  
   
### Messages sent to ESP32  
   
| Message | Description |  
|---------|-------------|  
| `ESP:PING:RECEIVE` | Heartbeat acknowledge |  
| `ESP:LED:ON/OFF` | LED state confirmation |  
| `ESP:ADC:BATT` | Battery measurement response |  
| `IMU:ON / IMU:OFF` | IMU activation state |  
| `IMU:SHAKE` | Shake gesture detected |  
| `IMU:Z%d,R%d,P%d` | Zone, roll x10, pitch x10 |  
| `STM:SLEEP` | Notifies ESP32 before entering sleep |  
| `STM:WAKE` | Notifies ESP32 after waking up |  
   
---  
   
## IMU — MPU6050  
   
Sampling rate: **20Hz (every 50ms)**    
Scale: **±4G**  
   
### Orientation Zones  
   
| Zone | Condition |  
|------|-----------|  
| `ZONE_FLAT` | No significant tilt |  
| `ZONE_RIGHT` | Roll > 45° |  
| `ZONE_LEFT` | Roll < -45° |  
| `ZONE_FORWARD` | Pitch > 30° |  
| `ZONE_BACK` | Pitch < -30° |  
| `ZONE_FACEDOWN` | Z axis < -0.8G |  
   
### Gesture Detection  
   
Shake detection uses a **circular buffer of 8 samples** calculating  
acceleration magnitude variance. A shake is triggered when:  
- Variance exceeds **0.35**  
- Cooldown of **1000ms** has elapsed since last shake  
   
---  
   
## Battery Measurement  
   
The battery voltage is measured using the internal **VREFINT** reference  
of the STM32 ADC for accurate VDD compensation.  
   
16 samples are averaged per measurement for noise reduction.  
   
---  
   
## OLED Interface  
   
| Screen | Trigger |  
|--------|---------|  
| Loading animation | System startup |  
| Main screen | Normal operation |  
| Fault screen | Error state |  
   
---  
   
## Build  
   
1. Open project in **STM32CubeIDE**  
2. Build with `Ctrl+B`  
3. Flash with `Run → Debug` or `Run → Run`  
   
---  
   
> ⚠️ Work in progress — active development.  
