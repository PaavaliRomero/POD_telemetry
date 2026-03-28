#include "pod_fsm.h"
#include "pod_shared.h"
#include "pod_imu.h"
#include "ui_oled.h"
#include "mpu6050.h"
#include "ssd1306.h"
#include "main.h"
#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdio.h>

/* ── Periféricos externos de main.c ─────────────── */
extern UART_HandleTypeDef huart1;
extern ADC_HandleTypeDef  hadc1;
extern MPU6050_Handle_t   mpu;

/* ── Variables externas de main.c ───────────────── */
extern volatile uint8_t cmd_ready;
extern char             cmd_buffer[];

/* ── Funciones externas de main.c ───────────────── */
extern void SystemClock_Config(void);

/* ── Defines internos ───────────────────────────── */
#define VREFINT_TYPICAL_mV 1200UL //Constante que permite realizar calulos de la bateria
#define PRESCALER_ADC_CAL 4095UL //Prescalador de la bateria fijo 2 (12)

uint16_t ADC_READ_CH(uint16_t channel, uint16_t sample_time)
{
	/*
	 * GET value of ADC1 but, get one channel value by one single medition.
	 * */

	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = channel;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = sample_time;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);

	/*Make average the samples of ADC*/
	uint32_t sum = 0;
	for	(int i = 0; i< 16;i++){
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		sum += HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
	}

	return (uint16_t)(sum / 16);

}

void POD_FSM_BATT_func(void)
{
	/*
	 * Calculate of voltage by battery.
	 * NOTES: Batery needs a voltage between 3V to 5V.
	 * */

	uint16_t ADC_vREFINT 	=	ADC_READ_CH(ADC_CHANNEL_VREFINT, ADC_SAMPLETIME_239CYCLES_5);
	uint32_t VDD_CONS_STM32	=	(VREFINT_TYPICAL_mV * PRESCALER_ADC_CAL)/ADC_vREFINT;
	uint16_t ADC_vBATT		=	ADC_READ_CH(ADC_CHANNEL_9, ADC_SAMPLETIME_239CYCLES_5);
	uint32_t Vpin_mV		=	(ADC_vBATT * VDD_CONS_STM32)/PRESCALER_ADC_CAL;
	VBATT = (uint16_t)(Vpin_mV * 2);

	HAL_UART_Transmit(&huart1, (uint8_t*)"ESP:ADC:BATT\n", strlen("ESP:ADC:BATT\n"), 100);
}

void POD_FSM_LED_func(void)
{
	/*
	 * Toggle GPIO_PIN_13 with a operation with CAST and send state of led.
	 * */

	  if (GPIOC->ODR & LED_Pin)
	  {
		  GPIOC->BSRR = (uint32_t)LED_Pin << 16;
		  State_LED = 1;
		  HAL_UART_Transmit(&huart1, (uint8_t*)"ESP:LED:ON\n", strlen("ESP:LED:ON\n"),100);
	  }
	  else
	  {
		  GPIOC->BSRR = LED_Pin;
		  State_LED = 0;
		  HAL_UART_Transmit(&huart1, (uint8_t*)"ESP:LED:OFF\n", strlen("ESP:LED:OFF\n"),100);
	  }
}

void POD_FSM_PING_func(void)
{
	/*
	 * Check comunication between STM32 and ESP32
	 *
	 * */

	HAL_UART_Transmit(&huart1, (uint8_t*)"ESP:PING:RECIVE\n", strlen("ESP:PING:RECIVE\n"), 100);
	HAL_Delay(100);
}

void POD_FSM_MPU_Toggle_func(void)
{
	/*Activate or desactiive function IMU*/
    imu_active = !imu_active;   /* ESP32 activa o desactiva el IMU */

    if (imu_active)
        HAL_UART_Transmit(&huart1, (uint8_t*)"IMU:ON\r\n", 8, 100);
    else
        HAL_UART_Transmit(&huart1, (uint8_t*)"IMU:OFF\r\n", 9, 100);
}

void POD_ModeSleep_Func(void)
{
    ssd1306_SetDisplayOn(0);
    HAL_UART_Transmit(&huart1, (uint8_t*)"STM:SLEEP\n", strlen("STM:SLEEP\n"), 100);
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
    // CPU pausa aquí hasta que presiones el botón
    SystemClock_Config();
    MPU6050_ConfigModifi(&mpu);
    ssd1306_SetDisplayOn(1);
    last_activity_tick = HAL_GetTick();
    HAL_UART_Transmit(&huart1, (uint8_t*)"STM:WAKE\n",
                      strlen("STM:WAKE\n"), 100);
}

void FINITE_STATE_MACHINE_MAIN(void)
{
	/*
	 * Finite-state machine (FMS) MAIN LOGIC
	 * */
	switch(pod_state)
	{
		case POD_STATE_INIT:

			if(HAL_GetTick() > 3000)
			{
				MPU6050_ConfigModifi(&mpu);
				POD_FSM_BATT_func();
				pod_state = POD_STATE_IDLE;
			}
		break;
		case POD_STATE_IDLE:

			pod_state_ui = POD_STATE_MainScreen;
		    uint32_t elapsed = HAL_GetTick() - last_activity_tick;

		    // Inactividad > 30s → apagar OLED únicamente por ahora
		    if (elapsed > 60000U && !imu_active) POD_ModeSleep_Func();

			if(cmd_ready)
			{
				cmd_ready = 0;
			    Clear_Screen_ssd1306();

				if		(strcmp(cmd_buffer, "STM:LED")==0)		POD_FSM_LED_func();
				else if	(strcmp(cmd_buffer,"STM:PING")==0)		POD_FSM_PING_func();
				else if	(strcmp(cmd_buffer, "STM:BATT")==0)		POD_FSM_BATT_func();
				else if	(strcmp(cmd_buffer, "STM:MPU")==0)		POD_FSM_MPU_Toggle_func();
				else	pod_state = POD_STATE_ERROR;
			}

		break;
		case POD_STATE_ERROR:
			pod_state_ui = POD_STATE_FaultScreen;
			if(cmd_ready)
			{
				cmd_ready = 0;
				pod_state = POD_STATE_IDLE;
				pod_state_ui = POD_STATE_MainScreen;
			}
		break;
	}
}

void POD_UI_TASK(void)
{
	/*
	 * FSM for print on the screen diferents states
	 *
	 * */
	switch(pod_state_ui)
	{
		case POD_STATE_AnimaLoad: POD_UI_AnimaLoad_Task(); break;
		case POD_STATE_MainScreen: POD_UI_MainScreen(); break;
		case POD_STATE_FaultScreen: POD_UI_FAULTSCREEN(); break;
	}

}
