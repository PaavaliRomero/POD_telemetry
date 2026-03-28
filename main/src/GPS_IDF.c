#include <stdio.h>
#include <string.h>
#include "driver/uart.h"
#include "esp_log.h"

#define baudRate 	      9600

void init_uart_config_GPS(uart_port_t portNum, int BufSize, int GpsTXpin, int GpsRXpin)
{
	
	/*CONFIG for init the comunication UART*/
	/*just only read. Not send message at device*/
	
    uart_config_t uart_config = {
        .baud_rate = baudRate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    uart_driver_install(portNum, BufSize * 2, 0, 0, NULL, 0);
    uart_param_config(portNum, &uart_config);
    uart_set_pin(portNum, GpsTXpin, GpsRXpin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}


void parse_gprmc(const char *sentence, const char *tag)
{
	
	/* SHOW LONG,TIME and LATITUDE of device*/
	
	
    char copy[128];
    strcpy(copy, sentence);

    char *token = strtok(copy, ",");
    int index = 0;

    char *time 		= 	NULL;
    char *status	= 	NULL;
    char *lat 		= 	NULL;
    char *lat_dir 	=	NULL;
    char *lon 		= 	NULL;
    char *lon_dir 	= 	NULL;

    while (token)
    {
        switch (index) {
            case 1: time = token; break;
            case 2: status = token; break;
            case 3: lat = token; break;
            case 4: lat_dir = token; break;
            case 5: lon = token; break;
            case 6: lon_dir = token; break;
        }
        token = strtok(NULL, ",");
        index++;
    }

    if (status && strcmp(status, "A") == 0) {
        ESP_LOGI(tag , "Lat: %s %s | Lon: %s %s | Time: %s\n", lat, lat_dir, lon, lon_dir,time);
    } else {
        ESP_LOGI(tag ,"No fix...\n");
    }
}
