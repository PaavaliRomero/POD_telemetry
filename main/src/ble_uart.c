#include "ble_uart.h"
#include "esp_bt_main.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_event.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_common_api.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "BLE_UART"; //mensaje para envio de consola.
static char s_device_name[32] = "POD-9S";

/* ── UUIDs del perfil Nordic UART Service (NUS) ─── */
#define NUS_SERVICE_UUID        0xFFF0
#define NUS_TX_CHAR_UUID        0xFFF1   // ESP32 → celular
#define NUS_RX_CHAR_UUID        0xFFF2   // celular → ESP32

/* ── Handles GATT ────────────────────────────────── */
static uint16_t s_gatts_if          = 0;
static uint16_t s_conn_id           = 0;
static uint16_t s_tx_char_handle    = 0;
static uint16_t s_cccd_handle       = 0;
static bool     s_connected         = false;
static bool     s_notify_enabled    = false;

/* ── Tabla de atributos GATT ─────────────────────── */
enum {
    IDX_SVC,
    IDX_TX_CHAR,
    IDX_TX_VAL,
    IDX_TX_CFG,      // CCCD para notificaciones
    IDX_RX_CHAR,
    IDX_RX_VAL,
    IDX_NB
};

static uint16_t s_handle_table[IDX_NB];

static const uint16_t primary_service_uuid   = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t char_decl_uuid         = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t char_config_uuid       = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t  char_prop_notify       = ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t  char_prop_write        = ESP_GATT_CHAR_PROP_BIT_WRITE |
                                               ESP_GATT_CHAR_PROP_BIT_WRITE_NR;

static const uint16_t nus_service_uuid       = NUS_SERVICE_UUID;
static const uint16_t nus_tx_uuid            = NUS_TX_CHAR_UUID;
static const uint16_t nus_rx_uuid            = NUS_RX_CHAR_UUID;
static const uint8_t  tx_value[1]            = {0};
static const uint8_t  rx_value[1]            = {0};
static const uint8_t  cccd_value[2]          = {0, 0};

static const esp_gatts_attr_db_t nus_gatt_db[IDX_NB] = {
    // Servicio primario
    [IDX_SVC] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid,
         ESP_GATT_PERM_READ,
         sizeof(uint16_t), sizeof(nus_service_uuid),
         (uint8_t *)&nus_service_uuid}
    },
    // TX characteristic declaration
    [IDX_TX_CHAR] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&char_decl_uuid,
         ESP_GATT_PERM_READ,
         sizeof(uint8_t), sizeof(uint8_t),
         (uint8_t *)&char_prop_notify}
    },
    // TX characteristic value
    [IDX_TX_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&nus_tx_uuid,
         ESP_GATT_PERM_READ,
         20, sizeof(tx_value),
         (uint8_t *)tx_value}
    },
    // TX CCCD (habilita notificaciones)
    [IDX_TX_CFG] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&char_config_uuid,
         ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
         sizeof(uint16_t), sizeof(cccd_value),
         (uint8_t *)cccd_value}
    },
    // RX characteristic declaration
    [IDX_RX_CHAR] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&char_decl_uuid,
         ESP_GATT_PERM_READ,
         sizeof(uint8_t), sizeof(uint8_t),
         (uint8_t *)&char_prop_write}
    },
    // RX characteristic value
    [IDX_RX_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&nus_rx_uuid,
         ESP_GATT_PERM_WRITE,
         20, sizeof(rx_value),
         (uint8_t *)rx_value}
    },
};

/* ── Advertising ─────────────────────────────────── */
static esp_ble_adv_params_t s_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static void start_advertising(void)
{
    esp_ble_gap_set_device_name(s_device_name);
    esp_ble_adv_data_t adv_data = {
        .set_scan_rsp    = false,
        .include_name    = true,
        .include_txpower = false,
        .flag            = ESP_BLE_ADV_FLAG_GEN_DISC |
                           ESP_BLE_ADV_FLAG_BREDR_NOT_SPT,
    };
    esp_ble_gap_config_adv_data(&adv_data);
}

/* ── GAP callback ────────────────────────────────── */
static void gap_event_handler(esp_gap_ble_cb_event_t event,
                               esp_ble_gap_cb_param_t *param)
{
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            esp_ble_gap_start_advertising(&s_adv_params);
            break;
        case ESP_GAP_BLE_SEC_REQ_EVT:
            esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
            break;
        case ESP_GAP_BLE_AUTH_CMPL_EVT:
            if (param->ble_security.auth_cmpl.success) {
                ESP_LOGI(TAG, "Pairing exitoso");
            } else {
                ESP_LOGE(TAG, "Pairing fallido");
                esp_ble_gap_start_advertising(&s_adv_params);
            }
            break;
        default:
            break;
    }
}

/* ── GATTS callback ──────────────────────────────── */
static void gatts_event_handler(esp_gatts_cb_event_t event,
                                 esp_gatt_if_t gatts_if,
                                 esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
        case ESP_GATTS_REG_EVT:
            s_gatts_if = gatts_if;
            esp_ble_gatts_create_attr_tab(nus_gatt_db, gatts_if,
                                          IDX_NB, 0);
            break;

        case ESP_GATTS_CREAT_ATTR_TAB_EVT:
            if (param->add_attr_tab.status == ESP_GATT_OK &&
                param->add_attr_tab.num_handle == IDX_NB) {
                memcpy(s_handle_table, param->add_attr_tab.handles,
                       sizeof(s_handle_table));
                s_tx_char_handle = s_handle_table[IDX_TX_VAL];
                s_cccd_handle    = s_handle_table[IDX_TX_CFG];
                esp_ble_gatts_start_service(s_handle_table[IDX_SVC]);
                start_advertising();
            }
            break;

        case ESP_GATTS_CONNECT_EVT:
            s_conn_id   = param->connect.conn_id;
            s_connected = true;
            ESP_LOGI(TAG, "Cliente conectado");
            ble_uart_send("[POD 042] LINK ESTABLISHED\n");
            break;

        case ESP_GATTS_DISCONNECT_EVT:
            s_connected      = false;
            s_notify_enabled = false;
            ESP_LOGI(TAG, "Cliente desconectado");
            esp_ble_gap_start_advertising(&s_adv_params);
            break;

		case ESP_GATTS_WRITE_EVT:
			if (param->write.handle == s_cccd_handle) {
				s_notify_enabled = (param->write.value[0] == 0x01);
				if (s_notify_enabled) {
					ESP_LOGI(TAG, "Notificaciones: ON");
					ble_uart_send("[POD 042] BOOTING TACTICAL SUPPORT UNIT...\n");
					ble_uart_send("[POD 042] SYSTEM ONLINE.\n");
				}
			}
			break;

        default:
            break;
    }
}

/* ── Inicialización ──────────────────────────────── */
esp_err_t ble_uart_init(const char *device_name)
{
    strncpy(s_device_name, device_name, sizeof(s_device_name) - 1);
    esp_err_t ret;

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) return ret;

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) return ret;

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret != ESP_OK) return ret;

    ret = esp_bluedroid_init();
    if (ret != ESP_OK) return ret;

    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) return ret;

    // whitout PIN - Just Works
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_NO_BOND;
    esp_ble_io_cap_t iocap      = ESP_IO_CAP_NONE;
    uint8_t key_size  = 16;
    uint8_t init_key  = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key   = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;

    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(auth_req));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE,      &iocap,    sizeof(iocap));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE,    &key_size, sizeof(key_size));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY,    &init_key, sizeof(init_key));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY,     &rsp_key,  sizeof(rsp_key));
	
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gatts_app_register(0);

    start_advertising();

    ESP_LOGI(TAG, "BLE UART iniciado");
    return ESP_OK;
}

/* ── Envío de telemetría ─────────────────────────── */
void ble_uart_send(const char *msg)
{
    if (!s_connected || !s_notify_enabled) return;
    if (s_gatts_if == 0 || s_tx_char_handle == 0) return;

    esp_ble_gatts_send_indicate(s_gatts_if, s_conn_id,
                                 s_tx_char_handle,
                                 strlen(msg), (uint8_t *)msg,
                                 false);
}

/* ── Estado de conexión ──────────────────────────── */
bool ble_uart_is_connected(void)
{
    return s_connected;
}

/* ── Deinicialización ────────────────────────────── */
void ble_uart_deinit(void)
{
    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
}
