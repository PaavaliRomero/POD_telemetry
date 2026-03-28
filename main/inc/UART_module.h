
#ifndef UART_MODULE_H
#define UART_MODULE_H

#include "driver/uart.h"
#include "driver/gpio.h"

#define UART_NUM UART_NUM_2	// CHANNEL
#define TXD_PIN 17 //RXD  A9	-> STM32
#define RXD_PIN 16 //TXD  A10	-> STM32

/** 
 * @brief Inicia el modulo de UART para la comunicacion con stm32.
 * 
 * @return ESP_OK inicializacion de uart exitosa.
 * @return ESP_FAIL fallo la inicializacion de uart.
 * 
 * @note se llama una sola vez al arrancar el sistema,
 * 		 preferente al inicio antes de entrar al bucle
 * 		 principal.
 * */
void init_UART_Module(void);

/** 
 * @brief Envia comandos al STM32.
 * 
 * @param *cmd: es un texto que esta codificado en stm32.
 * 
 * @return ESP_OK si se envio mensaje.
 * @return ESP_FAIL si el mensaje no se envio.
 * 
 * @note Procura que sea un texto pequeño para no saturar.
 * */
void pod_send_cmd_STM32(const char *cmd);

/** 
 * @brief lee comandos que son enviados por el stm32.
 * @brief Lee los comandos por caracteres.
 * 
 * @param *buf: arreglo donde recive el texto.
 * @param max_len: la extension del buffer.
 * @param timeout_ms: tiempo que requiere para lectura.
 * 
 * @return len: tamaño del contenido.
 * 
 * @note solo se llama una vez en el archivo pod_fsm.c.
 * 
 * @see pod_fsm_run(void). 
 * */
int pod_read_line(char *buf, int max_len, int timeout_ms);

#endif
