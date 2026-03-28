#pragma once
#include "esp_err.h"
#include <stdbool.h>

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
