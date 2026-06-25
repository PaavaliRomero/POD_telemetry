idf_component_register(
    SRCS    "pod_main.c"
            "src/modulo.c"
            "src/GPS_IDF.c"
            "src/UART_module.c"
            "src/pod_fsm.c"
            "src/ble_uart.c"
    INCLUDE_DIRS    "."
                    "inc"
                    "$ENV{IDF_PATH}/components/bt/include/esp32/include"
                    "$ENV{IDF_PATH}/components/bt/host/bluedroid/api/include/api"
    REQUIRES
        driver
        esp_driver_uart
        esp_driver_gpio
        esp_driver_ledc
        freertos
        bt
        nvs_flash
        esp_event
)

