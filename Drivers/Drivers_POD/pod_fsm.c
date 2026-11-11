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
extern ADC_HandleTypeDef  hadc1;
extern MPU6050_Handle_t   mpu;

/* ── Funciones externas de main.c ───────────────── */
extern void SystemClock_Config(void);

/* ── Defines internos ───────────────────────────── */
#define VREFINT_CAL_mV 1170UL
#define PRESCALER_ADC_CAL 4095UL //Prescalador de la bateria fijo 2 (12)
#define BATT_EN_SETTLE_MS 6U
#define ADC_READ_SAMPLES 16U //samples measures
#define ADC_CONVERSION_TIMEOUT_MS 2U

static void BATT_EN_On(void)
{
#if BATT_EN_ACTIVE_HIGH
	HAL_GPIO_WritePin(BATT_EN_GPIO_Port, BATT_EN_Pin, GPIO_PIN_SET);
#else
	HAL_GPIO_WritePin(BATT_EN_GPIO_Port, BATT_EN_Pin, GPIO_PIN_RESET);
#endif
}

static void BATT_EN_Off(void)
{
#if BATT_EN_ACTIVE_HIGH
	HAL_GPIO_WritePin(BATT_EN_GPIO_Port, BATT_EN_Pin, GPIO_PIN_RESET);
#else
	HAL_GPIO_WritePin(BATT_EN_GPIO_Port, BATT_EN_Pin, GPIO_PIN_SET);
#endif
}

/* LED activo en bajo: ON = pin a 0, OFF = pin a 1. */
static void POD_FSM_LED_Set(uint8_t on)
{
	if (on)
		GPIOC->BSRR = (uint32_t)LED_Pin << 16;
	else
		GPIOC->BSRR = LED_Pin;

	State_LED = on ? 1U : 0U;
}

static uint8_t level_led_was_active = 0;

static void POD_FSM_LED_UpdateLevel(void)
{
	if (!imu_active)
	{
		if (level_led_was_active && State_LED)
		{
			POD_FSM_LED_Set(0);
			POD_UART_Send("ESP:LED:OFF\n");
		}
		level_led_was_active = 0;
		return;
	}

	level_led_was_active = 1;

	uint8_t desired = IMU_Level ? 1U : 0U;
	if (desired != State_LED)
	{
		POD_FSM_LED_Set(desired);
		POD_UART_Send(desired ? "ESP:LED:ON\n" : "ESP:LED:OFF\n");
	}
}

static HAL_StatusTypeDef ADC_READ_CH(uint16_t channel, uint16_t sample_time, uint16_t *value)
{
	ADC_ChannelConfTypeDef sConfig = {0};

	if (value == NULL)
		return HAL_ERROR;

	sConfig.Channel = channel;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = sample_time;

	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		return HAL_ERROR;

	/*Make average the samples of ADC*/
	uint32_t sum = 0;
	for (uint32_t i = 0; i < ADC_READ_SAMPLES; i++)
	{
		if (HAL_ADC_Start(&hadc1) != HAL_OK)
			return HAL_ERROR;

		if (HAL_ADC_PollForConversion(&hadc1, ADC_CONVERSION_TIMEOUT_MS) != HAL_OK)
		{
			(void)HAL_ADC_Stop(&hadc1);
			return HAL_ERROR;
		}

		sum += HAL_ADC_GetValue(&hadc1);

		if (HAL_ADC_Stop(&hadc1) != HAL_OK)
			return HAL_ERROR;
	}

	*value = (uint16_t)(sum / ADC_READ_SAMPLES);

	return HAL_OK;
}

void POD_FSM_BATT_func(void)
{
	/*
	 * Calculate of voltage by battery.
	 * NOTES: Batery needs a voltage between 3V to 5V. resolve
	 * 		  Make a driver with two transistors and GPIPO_A8 able the driver
	 * */

	uint16_t adc_vrefint = 0;
	uint16_t adc_vbatt = 0;

	BATT_EN_On();
	HAL_Delay(BATT_EN_SETTLE_MS);

	if ((ADC_READ_CH(ADC_CHANNEL_VREFINT, ADC_SAMPLETIME_239CYCLES_5, &adc_vrefint) != HAL_OK) ||
	    (adc_vrefint == 0U))
	{
		BATT_EN_Off();
		POD_UART_Send("ESP:ADC:BATT:ERR\n");
		return;
	}

	uint32_t vdd_mv = (VREFINT_CAL_mV * PRESCALER_ADC_CAL) / adc_vrefint;

	if (ADC_READ_CH(ADC_CHANNEL_9, ADC_SAMPLETIME_239CYCLES_5, &adc_vbatt) != HAL_OK)
	{
		BATT_EN_Off();
		POD_UART_Send("ESP:ADC:BATT:ERR\n");
		return;
	}

	BATT_EN_Off();

	char raw_msg[80];
	int raw_len = snprintf(raw_msg, sizeof(raw_msg), "ESP:ADC:RAW:vref=%u,vbatt=%u,vdd=%u\n",
	                       (unsigned)adc_vrefint, (unsigned)adc_vbatt, (unsigned)vdd_mv);
	if (raw_len > 0 && raw_len < (int)sizeof(raw_msg))
		POD_UART_Send(raw_msg);

	uint32_t vpin_mv = ((uint32_t)adc_vbatt * vdd_mv) / PRESCALER_ADC_CAL;
	VBATT = (uint16_t)(vpin_mv * 2U);

	POD_UART_Send("ESP:ADC:BATT\n");

	char batt_msg[32];
	int batt_len = snprintf(batt_msg, sizeof(batt_msg), "ESP:ADC:BATT:%u\n", (unsigned)VBATT);
	if (batt_len > 0 && batt_len < (int)sizeof(batt_msg))
		POD_UART_Send(batt_msg);
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
		  POD_UART_Send("ESP:LED:ON\n");
	  }
	  else
	  {
		  GPIOC->BSRR = LED_Pin;
		  State_LED = 0;
		  POD_UART_Send("ESP:LED:OFF\n");
	  }
}

void POD_FSM_PING_func(void)
{
	/*
	 * Check comunication between STM32 and ESP32
	 *
	 * */

	POD_UART_Send("ESP:PING:RECEIVE\n");
}

void POD_FSM_MPU_Toggle_func(void)
{
	/*Activate or desactiive function IMU*/
    imu_active = !imu_active;   /* ESP32 activa o desactiva el IMU */

    if (imu_active)
    {
        POD_UART_Send("IMU:ON\r\n");
    }
    else
    {
        IMU_Level  = 0;
        IMU_Hold   = 0;
        IMU_Moving = 0;
        POD_UART_Send("IMU:OFF\r\n");
    }
}

void POD_ModeSleep_Func(void)
{
    BATT_EN_Off();
    ssd1306_SetDisplayOn(0);
    POD_UART_Send("STM:SLEEP\n");
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
    // CPU pausa aquí hasta que presiones el botón
    SystemClock_Config();
    POD_UART_Resume();
    MPU6050_ConfigModifi(&mpu);
    ssd1306_SetDisplayOn(1);
    last_activity_tick = HAL_GetTick();
    POD_UART_Send("STM:WAKE\n");
}

void FINITE_STATE_MACHINE_MAIN(void)
{
	/*
	 * Finite-state machine (FMS) MAIN LOGIC
	 * */
	char cmd[POD_CMD_BUF_SIZE];

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

			if(POD_UART_TakeCommand(cmd, sizeof(cmd)))
			{
				uint8_t user_activity = 0;

				if		(strcmp(cmd, "STM:LED")==0) {
					Clear_Screen_ssd1306();
					POD_FSM_LED_func();
					user_activity = 1;
				}
				else if	(strcmp(cmd,"STM:PING")==0)		POD_FSM_PING_func();
				else if	(strcmp(cmd, "STM:BATT")==0)		POD_FSM_BATT_func();
				else if	(strcmp(cmd, "STM:MPU")==0) {
					Clear_Screen_ssd1306();
					POD_FSM_MPU_Toggle_func();
					user_activity = 1;
				}
				else if	(strcmp(cmd, "STM:TARE")==0) {
					if (imu_active)
						IMU_Tare();
					user_activity = 1;
				}
				else if	(strcmp(cmd, "STM:HOLD")==0) {
					if (imu_active)
						IMU_HoldToggle();
					user_activity = 1;
				}
				else if	(strcmp(cmd, "STM:LIVE")==0) {
					if (imu_active) {
						IMU_LiveToggle();
						POD_UART_Send(IMU_LiveState() ? "ESP:LIVE:ON\n" : "ESP:LIVE:OFF\n");
					}
					user_activity = 1;
				}
				else	pod_state = POD_STATE_ERROR;

				if (user_activity)
					last_activity_tick = HAL_GetTick();
			}

			POD_FSM_LED_UpdateLevel();

			if (!POD_UART_HasPendingCommand() &&
				(HAL_GetTick() - last_activity_tick) > 60000U &&
				!imu_active)
				POD_ModeSleep_Func();

		break;
		case POD_STATE_ERROR:
			pod_state_ui = POD_STATE_FaultScreen;
			if(POD_UART_TakeCommand(cmd, sizeof(cmd)))
			{
				last_activity_tick = HAL_GetTick();
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
