/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "ssd1306.h"
#include "mpu6050.h"

#include "pod_shared.h"
#include "ui_oled.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define CMD_BUF_SIZE 64 //Tamaño del arreglo que requiere recibir.
#define MSG_LEN 48 //Tamaño del mensjae a enviar a ESP32
#define VREFINT_TYPICAL_mV 1200UL //Constante que permite realizar calulos de la bateria
#define PRESCALER_ADC_CAL 4095UL //Prescalador de la bateria fijo 2 (12)
#define ACCEL_SCALE_4G 8192.0f
/* ── IMU DEFINES ──────────────────────────── */
#define IMU_SAMPLE_MS    50U     // leer cada 50ms (20Hz)
#define IMU_ZONE_THRESH  45.0f   // grados para tilt lateral (roll)
#define IMU_PITCH_THRESH 30.0f   // grados para tilt adelante/atrás (pitch)
#define SHAKE_WINDOW 	 8 		//guarda las ultimas 8 muestras
#define SHAKE_THRESHOLD  0.35f   // varianza mínima para considerar shake
#define SHAKE_COOLDOWN   1000U   // ms entre un shake y el siguiente

/*── UART coneccition ──────────────────────── */
uint8_t rx_byte;
uint8_t Vbatt;

volatile uint8_t cmd_ready;
volatile uint8_t cmd_index = 0;

char cmd_buffer[CMD_BUF_SIZE];

/*── MPU variables ─────────────────────────── */
MPU6050_Handle_t mpu; //Sirve para el manejador del sensor.
uint8_t whoami;

/*── IMU STATE ─────────────────────────────── */
typedef enum {
    ZONE_FLAT     = 0,
    ZONE_RIGHT    = 1,
    ZONE_LEFT     = 2,
    ZONE_FORWARD  = 3,
    ZONE_BACK     = 4,
    ZONE_FACEDOWN = 5,
} IMU_Zone;

typedef enum {
    GESTURE_NONE  = 0,
    GESTURE_SHAKE = 1,
    GESTURE_FLIP  = 2,
} IMU_Gesture;

/* Flag: el ESP32 activó el IMU con STM:MPU */
volatile uint8_t  imu_active  = 0;
static 	 IMU_Zone imu_zone     = ZONE_FLAT;
static   IMU_Zone imu_zone_prev= ZONE_FLAT;
static   uint32_t imu_last_tick = 0;
static	 uint32_t last_activity_tick = 0;

/* Function DetectGesture */
static float    shake_buf[SHAKE_WINDOW];
static uint8_t  shake_idx     = 0;
static uint32_t shake_last    = 0;


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

void IMU_DetectGesture(float ax, float ay, float az)
{
    /* Magnitud total de la aceleración */
    float mag = sqrtf(ax*ax + ay*ay + az*az);


    /* Guardar en ventana circular */
    shake_buf[shake_idx] = mag;
    shake_idx = (shake_idx + 1) % SHAKE_WINDOW;


    /* Calcular varianza de la ventana */
    float mean = 0;
    for (int i = 0; i < SHAKE_WINDOW; i++) mean += shake_buf[i];
    mean /= SHAKE_WINDOW;

    float var = 0;
    for (int i = 0; i < SHAKE_WINDOW; i++)
        var += (shake_buf[i] - mean) * (shake_buf[i] - mean);
    var /= SHAKE_WINDOW;


    /* Shake detectado si varianza supera umbral */
    uint32_t now = HAL_GetTick();
    if (var > SHAKE_THRESHOLD &&
        (now - shake_last) > SHAKE_COOLDOWN)
    {
        shake_last = now;
        HAL_UART_Transmit(&huart1,
            (uint8_t*)"IMU:SHAKE\r\n", 11, 100);
    }

}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_3) {
        // Solo despertar — nada más
    }
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

void IMU_Processloop(void)
{
    if (!imu_active) return;
    if ((HAL_GetTick() - imu_last_tick) < IMU_SAMPLE_MS) return;
    imu_last_tick = HAL_GetTick();

	char msg[MSG_LEN];
	MPU6050_Accel_t accel;

	if (MPU6050_TestConnection(&mpu) != MPU6050_OK)
	{
		/*Check conection between devices*/

		sprintf(msg, "Device not ready\r\n");
		HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
		return;
	}

	if (MPU6050_GetAccelXYZ(&mpu, &accel) != MPU6050_OK) return;

	float ax_raw = accel.ax / ACCEL_SCALE_4G;
	float ay_raw = accel.ay / ACCEL_SCALE_4G;
	float az_raw = accel.az / ACCEL_SCALE_4G;

	float roll	= atan2f(ay_raw, az_raw) * 180.0f / M_PI;
	float pitch	= atan2f(-ax_raw, sqrtf(ay_raw*ay_raw + az_raw*az_raw)) * 180.0f / M_PI;

    if      (az_raw < -0.8f)             imu_zone = ZONE_FACEDOWN;
    else if (roll  >  IMU_ZONE_THRESH)   imu_zone = ZONE_RIGHT;
    else if (roll  < -IMU_ZONE_THRESH)   imu_zone = ZONE_LEFT;
    else if (pitch >  IMU_PITCH_THRESH)  imu_zone = ZONE_FORWARD;
    else if (pitch < -IMU_PITCH_THRESH)  imu_zone = ZONE_BACK;

    int16_t roll_i  = (int16_t)(roll  * 10.0f);
    int16_t pitch_i = (int16_t)(pitch * 10.0f);

    /* --- Transmisión continua: zona + valores numéricos --- */
    snprintf(msg, MSG_LEN, "IMU:Z%d,R%d,P%d\r\n", (int)imu_zone, roll_i, pitch_i);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);

    /* Solo transmitir si cambió la zona */
    if (imu_zone != imu_zone_prev) {
        imu_zone_prev = imu_zone;
    }
    imu_zone = imu_zone;

    /*This function detect gestures*/
    IMU_DetectGesture(ax_raw, ay_raw, az_raw);

    // Detección de movimiento por software
    float mag = sqrtf(ax_raw*ax_raw + ay_raw*ay_raw + az_raw*az_raw);
    if (fabsf(mag - 1.0f) > 0.3f)
        last_activity_tick = HAL_GetTick();
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
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
  ssd1306_Init();
  MPU6050_Init(&mpu, &hi2c2, MPU6050_I2C_ADDR);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  /* MAIN STATE MACHINE FOR LOGIC*/
	  FINITE_STATE_MACHINE_MAIN();
	  /* MAIN UI FSM for image in screen*/
	  POD_UI_TASK();
	  /*IMU commands for gestures*/
	  IMU_Processloop();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : INT_SLEEP_Pin */
  GPIO_InitStruct.Pin = INT_SLEEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(INT_SLEEP_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        if (rx_byte == '\n')
        {
        	cmd_buffer[cmd_index] = '\0';
        	cmd_ready = 1;
        	cmd_index = 0;
        }
        else
        {
        	if(cmd_index < sizeof(cmd_buffer)-1)
        	{
        		cmd_buffer[cmd_index++] = rx_byte;
        	}

        }
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);

    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
