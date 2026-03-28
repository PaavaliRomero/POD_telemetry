
#ifndef POD_FSM_H
#define POD_FSM_H

#define SWITCH_GPIO GPIO_NUM_4 // Pushboton

/** 
 * @brief Configuracion de FSM para el POD.
 * */
typedef enum {
    ESP_STATE_INIT = 0,   	//**< boot — check STM32 */
    ESP_STATE_IDLE,        	//**< Lisen for events STM32 */
    ESP_STATE_IMU_ACTIVE,  	//**< IMU active — Handle gestures and zones.  */
    ESP_STATE_ERROR,       	//**< STM32 dont response or error */
} ESP_PodState;

/** 
 * @brief Configuracion de zonas del IMU para POD.
 * @brief misma definición que STM32 
 * */
typedef enum {
    IMU_ZONE_FLAT     = 0, //**< Se encuentra en zona plana. */
    IMU_ZONE_RIGHT    = 1, //**< Giro a derecha. */
    IMU_ZONE_LEFT     = 2, //**< Giro a la izquierda. */
    IMU_ZONE_FORWARD  = 3, //**< Giro hacia el frente. */
    IMU_ZONE_BACK     = 4, //**< Giro hacia atras. */
    IMU_ZONE_FACEDOWN = 5, //**< Se encuentra boca abajo. */
} IMU_Zone;

/**
 * @brief Inicializa los pines que funcionan como in/out.
 *  
 * @return ESP_OK configuracion de GPIO exitosa.
 * @return ESP_FAIL fallo configuracion de GPIO.
 * 
 * @note Se inicializa solo una vez en main loop. 
 * */
void init_GPIO_Module(void);

/** 
 * @brief Inicializa la fsm de la logica del pod.
 * @brief Especificamente a la parte pensante del pod.
 * 
 * @return ESP_OK configuracion de fsm logica exitoso.
 * @return ESP_FAIL configuracion de fsm logica fallo.
 * 
 * @note Se inicializa solo una vez en main loop.
 * */
void pod_fsm_init(void);

/** 
 * @brief logica de el pod para enviar comandos, ejecutar uart, ble
 * 		  y gpios, ademas de funciones adicionales.
 * @brief llamar en cada iteración del loop.
 * 
 * @return Varias variables.
 * 
 * @note mantiene toda la logica por lo que recomienda si existe
 * 		 bugs o cualquier otro tipo de problemas, checar las funciones
 * 		 en especifico 
 * */
void pod_fsm_run(void);

#endif
