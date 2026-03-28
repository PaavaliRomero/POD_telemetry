
#include "UART_module.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "POD";

void init_UART_Module(void)
{
	/* Configuracion de UART2 para la comunicacion con el stm32 */
	uart_config_t uart_config = {
		.baud_rate = 115200,
		.data_bits = UART_DATA_8_BITS,
		.parity    = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
	};

	uart_driver_install(UART_NUM, 1024, 1024, 0, NULL, 0);
	uart_param_config(UART_NUM, &uart_config);
	uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void pod_send_cmd_STM32(const char *cmd)
{	
	/* Send a message to STM32 */
	uart_write_bytes(UART_NUM, cmd, strlen(cmd));
    ESP_LOGI(TAG, "TX:	%s",cmd);
}

int pod_read_line(char *buf, int max_len, int timeout_ms)
{
	/* Read the message of STM32 */
    int len = 0;
    uint8_t c;

    while (len < max_len - 1) {
        int ret = uart_read_bytes(UART_NUM, &c, 1, pdMS_TO_TICKS(timeout_ms));
        if (ret <= 0) break;        // timeout without data
        if (c == '\r') continue;    // skip \r
        if (c == '\n') break;       // End to line, get out line
        buf[len++] = (char)c;
    }

    buf[len] = '\0';
    return len;
}
