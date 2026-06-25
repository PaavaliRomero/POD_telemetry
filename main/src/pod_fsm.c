
#include "pod_fsm.h"
#include "ble_uart.h"
#include "UART_module.h"
#include "esp_log.h"
#include <string.h>

#define TAG "POD_FSM"
#define BUF_SIZE   128 /* buffer size */

#define PING_MS    3000U   /* heartbeat each 3s */
static uint32_t    last_ping  = 0;

#define BATT_MS  30000U   /* time for send comand of read battery. */
static uint32_t last_batt = 0; 
static uint8_t flag_batt = 1;

static ESP_PodState state = ESP_STATE_INIT;
static char rx[BUF_SIZE];

static uint8_t imu_active = 0;   // 0 = sleep, 1 = active

/* ── Init GPIOs ────────────────────────────────────── */
void init_GPIO_Module(void)
{
	/* Configuracion de pin para pushboton de control */
	gpio_config_t sw_cfg = {
    .pin_bit_mask = (1ULL << SWITCH_GPIO),
    .mode         = GPIO_MODE_INPUT,
    .pull_up_en   = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type    = GPIO_INTR_DISABLE,
	};
	
	gpio_config(&sw_cfg);
}

/* ── Message parser for STM32 ──────────────────────── */

static void handle_rx(const char *msg)
{	
    if (strcmp(msg, "ESP:PING:RECEIVE") == 0)
        ESP_LOGI(TAG, "[HB]  STM32 alive");

    else if (strcmp(msg, "ESP:LED:ON")  == 0)
        ESP_LOGI(TAG, "[LED] ON");

    else if (strcmp(msg, "ESP:LED:OFF") == 0)
        ESP_LOGI(TAG, "[LED] OFF");

    else if (strcmp(msg, "ESP:ADC:BATT") == 0) {
        ESP_LOGI(TAG, "[BAT] respuesta recibida");
         ble_uart_send("[POD 042] POWER CELL: OK\n");
	}

    else if (strcmp(msg, "IMU:ON") == 0) {
        imu_active = 1;
        state = ESP_STATE_IMU_ACTIVE;
        ESP_LOGI(TAG, "[IMU] activado");
		ble_uart_send("[POD 042] OPERATOR STATUS: ACTIVE\n");
    }
    else if (strcmp(msg, "IMU:OFF") == 0) {
        imu_active = 0;
        state = ESP_STATE_IDLE; 
        ESP_LOGI(TAG, "[IMU] desactivado");
        ble_uart_send("[POD 042] OPERATOR STATUS: IDLE\n");
    }

    // SHAKE — IMU toggle
    else if (strcmp(msg, "IMU:SHAKE") == 0) {
        pod_send_cmd_STM32("STM:MPU\n");   // toggle en STM32
        ESP_LOGI(TAG, "[IMU] %s", imu_active ? "*** SHAKE — apagando ***"
                                             : "*** SHAKE — encendiendo ***");
    }

    // Zones — Run it only if the IMU is active.
	else if (strncmp(msg, "IMU:Z", 5) == 0) {
		static int zona_prev = -1;
		int zona   = 0;
		int roll_i = 0;
		int pitch_i = 0;

		// New format parser: IMU:Z%d,R%d,P%d
		int parsed = sscanf(msg, "IMU:Z%d,R%d,P%d", &zona, &roll_i, &pitch_i);

		// If the zone sends the old format, do it the same way.
		float roll  = (parsed >= 2) ? roll_i  / 10.0f : 0.0f;
		float pitch = (parsed >= 3) ? pitch_i / 10.0f : 0.0f;

		switch (zona) {
			case 0:  // NEUTRAL
				ESP_LOGI(TAG, "[IMU] NEUTRAL — idle (R:%.1f P:%.1f)", roll, pitch);
				break;
			case 1:  // RIGHT
				ESP_LOGI(TAG, "[IMU] RIGHT (R:%.1f P:%.1f)", roll, pitch);
				pod_send_cmd_STM32("STM:LED\n"); 
				break;
			case 2:  // LEFT
				ESP_LOGI(TAG, "[IMU] LEFT (R:%.1f P:%.1f)", roll, pitch); 
				break;
			case 3:  // FORWARD
				ESP_LOGI(TAG, "[IMU] FORWARD (R:%.1f P:%.1f)", roll, pitch);
				pod_send_cmd_STM32("STM:PING\n"); 
				break;
			case 4:  // BACK
				ESP_LOGI(TAG, "[IMU] BACK (R:%.1f P:%.1f)", roll, pitch);
				pod_send_cmd_STM32("STM:MPU\n"); 
				break;
			case 5:  // FACEDOWN
				ESP_LOGI(TAG, "[IMU] FACEDOWN — reservado"); 
				break;
			default:
				ESP_LOGW(TAG, "[IMU] zona desconocida: %d", zona);
				break;
		}
		
		// Telemetría solo cuando cambia la zona
		if (zona != zona_prev) {
			zona_prev = zona;
			switch (zona) {
        case 0: ble_uart_send("[POD 042] ORIENTATION: VECTOR NEUTRAL\n"); break;
        case 1: ble_uart_send("[POD 042] ORIENTATION: VECTOR RIGHT\n");   break;
        case 2: ble_uart_send("[POD 042] ORIENTATION: VECTOR LEFT\n");    break;
        case 3: ble_uart_send("[POD 042] ORIENTATION: VECTOR FORWARD\n"); break;
        case 4: ble_uart_send("[POD 042] ORIENTATION: VECTOR BACK\n");    break;
				case 5: ble_uart_send("[POD 042] ALERT: UNIT INVERTED. RECOMMEND REPOSITION.\n"); break;
				default: break;
			}
		}
	}
    else
        ESP_LOGW(TAG, "[??] '%s'", msg);
}

/* ── FSM init ─────────────────────────────────── */
void pod_fsm_init(void)
{
    state = ESP_STATE_INIT;
    last_ping = xTaskGetTickCount() * portTICK_PERIOD_MS;
    last_batt = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    ble_uart_init("POD-9S");
    ble_uart_send("[POD 042] SYSTEM ONLINE\n");
}

void pod_fsm_run(void)
{
    // Read STM32.
    int len = pod_read_line(rx, BUF_SIZE, 10);
    if (len > 0) handle_rx(rx);

    // Read button.
    static uint8_t sw_last = 1;   // 1 = open (pull-up)
    uint8_t sw_now = gpio_get_level(SWITCH_GPIO);

    if (sw_last == 1 && sw_now == 0) {
        // falling endge — toogle do it.
        pod_send_cmd_STM32("STM:MPU\n");
    }
    sw_last = sw_now;

    switch (state)
    {
        case ESP_STATE_INIT:
            pod_send_cmd_STM32("STM:PING\n");
            state = ESP_STATE_IDLE;
        break;

        case ESP_STATE_IDLE:
        //Heartbeat by 3s
            if (xTaskGetTickCount() * portTICK_PERIOD_MS - last_ping > PING_MS) {
                pod_send_cmd_STM32("STM:PING\n");
                last_ping = xTaskGetTickCount() * portTICK_PERIOD_MS;
            }
        //Get batery by 30s
            if (flag_batt || xTaskGetTickCount() * portTICK_PERIOD_MS - last_batt > BATT_MS)
			{
				flag_batt = 0;
				ESP_LOGI(TAG, "[BAT] solicitando...");
				pod_send_cmd_STM32("STM:BATT\n");
				last_batt = xTaskGetTickCount() * portTICK_PERIOD_MS;
			}
        break;

        case ESP_STATE_IMU_ACTIVE:
            /* handle_rx() process IMU:Zx / auto-IMU:SHAKE */
        break;

        case ESP_STATE_ERROR:
			ble_uart_send("[POD 042] WARNING: SYSTEM FAULT DETECTED.\n");
            ESP_LOGE(TAG, "STM32 error — reintentando...");
            vTaskDelay(pdMS_TO_TICKS(1000));
            pod_send_cmd_STM32("STM:PING\n");
            state = ESP_STATE_IDLE;
        break;
    }
}
