
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "led_strip.h"
#include "sdkconfig.h"

#include "UART_module.h"
#include "pod_fsm.h"

void app_main(void)
{
	
	init_UART_Module();
	init_GPIO_Module();
	pod_fsm_init();
	
    vTaskDelay(pdMS_TO_TICKS(500));
	
    // Loop vacío
    while (1) {
	    pod_fsm_run();
        vTaskDelay(pdMS_TO_TICKS(10)); /* 10ms — no bloquea */
    }
}




