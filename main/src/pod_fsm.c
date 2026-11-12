
#include "pod_fsm.h"
#include "ble_uart.h"
#include "UART_module.h"
#include "esp_log.h"
#include <ctype.h>
#include <stdio.h>
#include <string.h>

#define TAG "POD_FSM"
#define BUF_SIZE   128 /* buffer size */

#define PING_MS    5000U   /* heartbeat each 5s */
static uint32_t    last_ping  = 0;

#define BATT_MS  30000U   /* time for send comand of read battery. */
static uint32_t last_batt = 0;
static uint8_t flag_batt = 1;

#define IMU_TELEMETRY_MS  500U   /* 2 Hz telemetry to BLE */
/* 1000 ms = 1 Hz (legible). Bajar a 100 para capturar ruido a 10 Hz. */
#define IMU_LOG_MS        100U // 1000U
static uint32_t last_imu_tx = 0;
static uint32_t last_imu_rx = 0;
static uint32_t last_imu_log = 0;

/* ── Button timing ─────────────────────────────────── */
#define BTN_DEBOUNCE_MS   30U
#define BTN_LONG_MS       800U   /* long press → HOLD */
#define BTN_DOUBLE_MS     500U   /* double click window → MPU toggle */

static ESP_PodState state = ESP_STATE_INIT;
static char rx[BUF_SIZE];

static uint8_t imu_active = 0;      // 0 = sleep, 1 = active
static volatile uint8_t stm_sleeping = 0;

/* Last telemetry received from the STM32. */
static int      imu_level   = 0;
static int      imu_roll_i  = 0;
static int      imu_pitch_i = 0;
static int      imu_hold_i  = 0;   /* 1 = valor congelado por auto/manual hold */
static int      imu_mov_i   = 0;   /* 1 = el STM32 detecta movimiento */
static uint8_t  imu_have_data = 0;

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

/* ── Helpers ───────────────────────────────────────── */

static uint32_t pod_now_ms(void)
{
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

/*
 * Send a command to the STM32 only if it is awake.
 * Commands sent while it is in STOP are lost (UART is not armed yet).
 */
static uint8_t pod_cmd_if_awake(const char *cmd, const char *label)
{
    if (stm_sleeping) {
        ESP_LOGW(TAG, "[CMD] STM32 dormido, se descarta: %s", label);
        return 0;
    }

    pod_send_cmd_STM32(cmd);
    ESP_LOGI(TAG, "[BTN] %s", label);

    char note[24];
    snprintf(note, sizeof(note), "[POD 042] BTN %.4s\n", label);
    ble_uart_send(note);
    return 1;
}

/* ── Telemetry to BLE ──────────────────────────────── */

static void pod_send_imu_telemetry(void)
{
    char line[32];

    /* Worst case: "IMU R-1800 P-900 L0\n" = 20 bytes (default ATT MTU). */
    snprintf(line, sizeof(line), "IMU R%d P%d L%d\n",
             imu_roll_i, imu_pitch_i, imu_level);

    ble_uart_send(line);
}

/* ── BLE RX → STM32 forwarding ─────────────────────── */

static void ble_rx_handler(const char *data, uint16_t len)
{
    char cmd[32];
    size_t n = 0;

    /* Trim CR/LF/spaces on both ends. */
    while (len > 0 && (data[0] == '\r' || data[0] == '\n' ||
                       data[0] == ' '  || data[0] == '\t' ||
                       data[0] == '\0')) {
        data++;
        len--;
    }
    while (len > 0 && (data[len - 1] == '\r' || data[len - 1] == '\n' ||
                       data[len - 1] == ' '  || data[len - 1] == '\t' ||
                       data[len - 1] == '\0')) {
        len--;
    }

    if (len == 0)
        return;

    /* Copy in uppercase and ignore a leading "STM:". */
    for (size_t i = 0; i < len && n < (sizeof(cmd) - 1); i++) {
        char c = (char)toupper((unsigned char)data[i]);
        if (c == '\0' || c == '\r' || c == '\n')
            break;
        cmd[n++] = c;
    }
    cmd[n] = '\0';

    const char *body = cmd;
    if (strncmp(body, "STM:", 4) == 0)
        body += 4;

    /* Only accept commands the STM32 knows; anything else puts it in ERROR. */
    if (strcmp(body, "TARE") != 0 && strcmp(body, "TARECLR") != 0 &&
        strcmp(body, "HOLD") != 0 &&
        strcmp(body, "MPU")  != 0 && strcmp(body, "PING") != 0 &&
        strcmp(body, "BATT") != 0 && strcmp(body, "LED")  != 0 &&
        strcmp(body, "LIVE") != 0) {
        ESP_LOGW(TAG, "[BLE] comando no valido: '%s'", cmd);
        ble_uart_send("[POD 042] CMD INVALID\n");
        return;
    }

    if (stm_sleeping) {
        ESP_LOGW(TAG, "[BLE] STM32 dormido, se descarta: %s", body);
        ble_uart_send("[POD 042] STM SLEEPING\n");
        return;
    }

    char out[40];
    snprintf(out, sizeof(out), "STM:%s\n", body);
    ESP_LOGI(TAG, "[BLE] TX %s", out);
    pod_send_cmd_STM32(out);

    char note[24];
    snprintf(note, sizeof(note), "[POD 042] CMD %.8s\n", body);
    ble_uart_send(note);
}

/* ── Message parser for STM32 ──────────────────────── */

static void handle_rx(const char *msg)
{
    uint32_t now = pod_now_ms();

    if (strcmp(msg, "STM:SLEEP") != 0)
        stm_sleeping = 0;

    if (strcmp(msg, "ESP:PING:RECEIVE") == 0) {
        stm_sleeping = 0;
        ESP_LOGI(TAG, "[HB]  STM32 alive");
    }
    else if (strcmp(msg, "STM:SLEEP") == 0) {
        stm_sleeping = 1;
        imu_active   = 0;
        imu_have_data = 0;
        state        = ESP_STATE_IDLE;
        ESP_LOGI(TAG, "[PWR] STM32 en STOP");
        ble_uart_send("[POD 042] STM: SLEEP\n");
    }
    else if (strcmp(msg, "STM:WAKE") == 0) {
        stm_sleeping = 0;
        last_ping = now;
        last_batt = now;
        flag_batt = 1;
        ESP_LOGI(TAG, "[PWR] STM32 despierto");
        ble_uart_send("[POD 042] STM: WAKE\n");
    }

    else if (strcmp(msg, "ESP:LED:ON")  == 0)
        ESP_LOGI(TAG, "[LED] ON");

    else if (strcmp(msg, "ESP:LED:OFF") == 0)
        ESP_LOGI(TAG, "[LED] OFF");

    else if (strcmp(msg, "ESP:ADC:BATT") == 0) {
        ESP_LOGI(TAG, "[BAT] respuesta recibida");
    }
    else if (strncmp(msg, "ESP:ADC:BATT:", 13) == 0) {
        const char *val = msg + 13;
        unsigned mv = 0;

        if (strcmp(val, "ERR") == 0) {
            ESP_LOGW(TAG, "[BAT] error de lectura");
            ble_uart_send("[POD 042] BATT: ERR\n");
        }
        else if (sscanf(val, "%u", &mv) == 1) {
            char out[32];

            ESP_LOGI(TAG, "[BAT] %u mV", mv);
            snprintf(out, sizeof(out), "[POD 042] BATT: %u mV\n", mv);
            ble_uart_send(out);
        }
        else {
            ESP_LOGW(TAG, "[BAT] formato invalido: '%s'", msg);
        }
    }
    else if (strncmp(msg, "ESP:ADC:RAW:", 12) == 0) {
        unsigned vref = 0, vbatt = 0, vdd = 0;

        if (sscanf(msg, "ESP:ADC:RAW:vref=%u,vbatt=%u,vdd=%u",
                   &vref, &vbatt, &vdd) == 3) {
            ESP_LOGI(TAG, "[ADC] vref=%u vbatt=%u vdd=%u", vref, vbatt, vdd);
        }
        else {
            ESP_LOGW(TAG, "[ADC] RAW invalido: '%s'", msg);
        }
    }

    else if (strcmp(msg, "IMU:ON") == 0) {
        imu_active    = 1;
        imu_have_data = 0;      /* waits ~2.5 s for gyro bias calibration */
        last_imu_rx   = now;
        state         = ESP_STATE_IMU_ACTIVE;
        ESP_LOGI(TAG, "[IMU] activado");
        ble_uart_send("[POD 042] OPERATOR STATUS: ACTIVE\n");
    }
    else if (strcmp(msg, "IMU:OFF") == 0) {
        imu_active    = 0;
        imu_have_data = 0;
        imu_level     = 0;
        state         = ESP_STATE_IDLE;
        ESP_LOGI(TAG, "[IMU] desactivado");
        ble_uart_send("[POD 042] OPERATOR STATUS: IDLE\n");
    }

    /*
     * Level telemetry from STM32:
     *   "IMU:L<level>,R<roll*10>,P<pitch*10>,H<hold>,M<moving>"
     * Example: "IMU:L0,R49,P8,H0,M0" → 4.9° roll, 0.8° pitch, not level,
     * live reading (H0) and no motion detected (M0).
     * The H/M fields are optional: older firmware sends only L,R,P.
     */
    else if (strncmp(msg, "IMU:L", 5) == 0) {
        int lvl = 0;
        int roll_i = 0;
        int pitch_i = 0;
        int hold_i = 0;
        int mov_i = 0;

        int fields = sscanf(msg, "IMU:L%d,R%d,P%d,H%d,M%d",
                            &lvl, &roll_i, &pitch_i, &hold_i, &mov_i);

        if (fields >= 3) {
            imu_level     = (lvl != 0) ? 1 : 0;
            imu_roll_i    = roll_i;
            imu_pitch_i   = pitch_i;
            imu_hold_i    = (fields >= 4) ? ((hold_i != 0) ? 1 : 0) : 0;
            imu_mov_i     = (fields >= 5) ? ((mov_i  != 0) ? 1 : 0) : 0;
            imu_have_data = 1;
            last_imu_rx   = now;

            if (!imu_active) {
                /* Re-sync if an IMU:ON notification was missed. */
                imu_active = 1;
                state      = ESP_STATE_IMU_ACTIVE;
                ESP_LOGI(TAG, "[IMU] activado (telemetria)");
            }

            /* Visible on the serial terminal to validate the link. */
            if ((now - last_imu_log) >= IMU_LOG_MS) {
                last_imu_log = now;
                ESP_LOGI(TAG, "[IMU] R=%.1f P=%.1f L=%d H=%d M=%d BLE=%s",
                         roll_i / 10.0f, pitch_i / 10.0f, imu_level,
                         imu_hold_i, imu_mov_i,
                         ble_uart_is_notifying() ? "on" : "off");
            }
        }
        else {
            ESP_LOGW(TAG, "[IMU] telemetria invalida: '%s'", msg);
        }
    }
    /*
     * I2C bus recovery report from STM32. It fires when the sensor did
     * not answer for several consecutive samples.
     *   "IMU:I2C:RECOVER" -> bus cleared and MPU reconfigured
     *   "IMU:I2C:FAIL"    -> SDA/SCL still stuck or MPU did not answer
     */
    else if (strcmp(msg, "IMU:I2C:RECOVER") == 0) {
        ESP_LOGW(TAG, "[I2C] bus recuperado");
        ble_uart_send("[POD 042] I2C RECOVER\n");
    }
    else if (strcmp(msg, "IMU:I2C:FAIL") == 0) {
        ESP_LOGE(TAG, "[I2C] no se pudo recuperar el bus");
        ble_uart_send("[POD 042] I2C FAIL\n");
    }
    else if (strncmp(msg, "ESP:LIVE:", 9) == 0) {
        ESP_LOGI(TAG, "[LIVE] %s", msg + 9);
        if (strcmp(msg + 9, "ON") == 0)
            ble_uart_send("[POD 042] LIVE ON\n");
        else
            ble_uart_send("[POD 042] LIVE OFF\n");
    }
    else if (strcmp(msg, "ESP:TARE:CLEAR") == 0) {
        ESP_LOGI(TAG, "[TARE] cero borrado");
        ble_uart_send("[POD 042] TARE CLEAR\n");
    }
    else if (strcmp(msg, "ESP:TARE:ERR") == 0) {
        ESP_LOGW(TAG, "[TARE] no se pudo borrar el cero");
        ble_uart_send("[POD 042] TARE ERR\n");
    }

    else
        ESP_LOGW(TAG, "[??] '%s'", msg);
}

/* ── Button: short = TARE, long = HOLD, double = MPU ─ */

static void button_task(uint32_t now)
{
    uint8_t raw = gpio_get_level(SWITCH_GPIO);

    static uint8_t  btn_raw_last      = 1;
    static uint8_t  btn_stable        = 1;
    static uint32_t btn_raw_change_ms = 0;
    static uint32_t btn_press_ms      = 0;
    static uint32_t btn_release_ms    = 0;
    static uint8_t  btn_long_fired    = 0;
    static uint8_t  btn_clicks        = 0;

    /* Debounce the raw level. */
    if (raw != btn_raw_last) {
        btn_raw_last      = raw;
        btn_raw_change_ms = now;
    }

    if (raw != btn_stable &&
        (now - btn_raw_change_ms) >= BTN_DEBOUNCE_MS) {
        btn_stable = raw;

        if (btn_stable == 0) {
            /* Press. */
            btn_press_ms   = now;
            btn_long_fired = 0;
        }
        else {
            /* Release. */
            btn_release_ms = now;

            if (!btn_long_fired) {
                if (btn_clicks == 0) {
                    btn_clicks = 1;      /* wait for possible second click */
                }
                else {
                    btn_clicks = 0;
                    pod_cmd_if_awake("STM:MPU\n", "MPU");
                }
            }
            btn_long_fired = 0;
        }
    }

    /* Long press: fire HOLD once, before releasing. */
    if (btn_stable == 0 && !btn_long_fired &&
        (now - btn_press_ms) >= BTN_LONG_MS) {
        btn_long_fired = 1;
        btn_clicks     = 0;
        pod_cmd_if_awake("STM:HOLD\n", "HOLD");
    }

    /* Single click: TARE after the double-click window expires.
     * Do not fire while a new press is in progress (it may become a
     * long press or the second click of a double click). */
    if (btn_stable == 1 && btn_clicks == 1 &&
        (now - btn_release_ms) >= BTN_DOUBLE_MS) {
        btn_clicks = 0;
        pod_cmd_if_awake("STM:TARE\n", "TARE");
    }
}

/* ── FSM init ─────────────────────────────────── */
void pod_fsm_init(void)
{
    state = ESP_STATE_INIT;
    last_ping = pod_now_ms();
    last_batt = last_ping;

    ESP_LOGI(TAG, "[BOOT] POD-9S build %s %s", __DATE__, __TIME__);

    ble_uart_init("POD-9S");
    ble_uart_set_rx_callback(ble_rx_handler);
    ble_uart_send("[POD 042] SYSTEM ONLINE\n");
}

void pod_fsm_run(void)
{
    uint32_t now = pod_now_ms();

    // Read STM32.
    int len = pod_read_line(rx, BUF_SIZE, 10);
    if (len > 0) handle_rx(rx);

    // Read button (short / long / double click).
    button_task(now);

    if (!stm_sleeping && now - last_ping > PING_MS) {
        pod_send_cmd_STM32("STM:PING\n");
        last_ping = now;
    }

    if (!stm_sleeping && (flag_batt || now - last_batt > BATT_MS)) {
        flag_batt = 0;
        ESP_LOGI(TAG, "[BAT] solicitando...");
        pod_send_cmd_STM32("STM:BATT\n");
        last_batt = now;
    }

    // Telemetry to BLE at 2 Hz while the IMU is active and fresh.
    if (imu_active && !stm_sleeping && imu_have_data &&
        (now - last_imu_rx) < 1000U &&
        (now - last_imu_tx) >= IMU_TELEMETRY_MS) {
        last_imu_tx = now;
        pod_send_imu_telemetry();
    }

    switch (state)
    {
        case ESP_STATE_INIT:
            pod_send_cmd_STM32("STM:PING\n");
            state = ESP_STATE_IDLE;
        break;

        case ESP_STATE_IDLE:
        break;

        case ESP_STATE_IMU_ACTIVE:
            /* Telemetry is sent above; parser updates the level state. */
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
