#pragma once
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

/** 
 * @brief Inicia el stack de BLE y comienza advertising del POD.
 * 		  Configurar el stack Bluedroid, registra los callbacks
 * 		  GAP y GATTS y comienza a transimitir el nombre del
 * 		  dispositivo para que el celular pueda descubrirlo.
 * 
 * @param *device_name: nombre visible para identificar el dispositivo.
 * 						Por defecto usara el nombre "POD-9S".
 * 
 * @return ESP_OK es la inicializacion exitosa.
 * @return ESP_FAIL fallo la inicializacion.
 * 
 * @note Debe llamarse solo una vez al arrancar el sistema, preferentemente
 * 		 en pod_fsm_init()
 * @note requiere que NVS flash este disponible.
 * */
esp_err_t ble_uart_init(const char *device_name);

/** 
 * @brief Envia un mensaje de telemetria al celular.
 * 		  El mensaje solo se envia cuando el dispositivo esta conectado
 * 		  y CCCD activo.
 * 
 * @param *msg: Cadena de texto a enviar. El mensaje sera truncado
 * 				a mas de 20 caracteres.
 * 
 * @see ble_uart_is_connected()
 * */
void      ble_uart_send(const char *msg);

/**
 * @brief Indica si hay un cliente conectado con notificaciones activas.
 *
 * @return true si la telemetria por BLE puede llegar al celular.
 * */
bool      ble_uart_is_notifying(void);

/**
 * @brief Registra el callback que recibe los datos escritos por el
 *        cliente BLE en la caracteristica RX (0xFFF2).
 *
 * @param cb Funcion llamada con el buffer y su longitud. No se agrega
 *           terminador '\0'; el callback debe copiar/validar los datos.
 *
 * @note Permite reenviar comandos del celular al STM32.
 * */
typedef void (*ble_uart_rx_cb_t)(const char *data, uint16_t len);
void      ble_uart_set_rx_callback(ble_uart_rx_cb_t cb);

/** 
 * @brief Verifica la actualizacion del cliente
 * 
 * @return true: si hay conexion activa y CCCD activado.
 * @return false: si no existe cliente conectado.
 * */
bool      ble_uart_is_connected(void);

/** 
 * @brief Desinicializa el stack de BLE y libera recursos.
 * 
 * @note declarar esta funcion sin haber llamado a ble_uart_init()
 * 		 causa un crasheo en el sistema.
 * 
 * @warning Despues de llamar esta funcion el dispositivo no sera 
 * 			visible por BLE hasta reiniciar el sistema.
 * 
 * */
void      ble_uart_deinit(void);
