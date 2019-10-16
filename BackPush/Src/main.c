/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include<stdio.h>


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
ADC_HandleTypeDef hadc;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

extern __IO uint32_t uwTick;
typedef enum
{
	off = 0,
	one,
	two
}Gears;

typedef enum
{
	stop = 0,
	standby,
	on
}WorkStatus;


#define constrain(amt,low,high)  ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define ARRAY_NUMBER 50
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim1;
uint32_t ADC_Value[ARRAY_NUMBER],Volt_Sum,Volt_average;
float Volt_Real;
uint8_t key1_status,key2_status;
uint16_t pwm_value;
uint8_t i;
uint32_t runtimes;
uint32_t key1_count,key2_count;
uint8_t  key1_loosen,key2_loosen;
uint16_t pwm_target;
float pwm_temp;
uint8_t low_power_flag;
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
  MX_ADC_Init();
  MX_TIM3_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
//	HAL_ADC_Start_DMA(&hadc1,(uint32_t *)&ADC_Value,100);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim16,TIM_CHANNEL_1);
	HAL_GPIO_WritePin(ESC_POW_EN_GPIO_Port,ESC_POW_EN_Pin,GPIO_PIN_RESET);
	//initialization electronic speed controller
//	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,100);
//	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,100);
//	HAL_Delay(1000);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		//key1 开关键 key2 速度键
//		key1_status = HAL_GPIO_ReadPin(GPIOA,KEY1_Pin);
//		key2_status = HAL_GPIO_ReadPin(GPIOA,KEY2_Pin);
		
		if(!HAL_GPIO_ReadPin(GPIOA,KEY1_Pin))
		{
			key1_count++;
		}
		else 
		{
			key1_count = 0;	
			key1_loosen = 1;			
		}
		
		
		
		if(key1_status == stop)
		{
			if(key1_count >= 300)
			{
				if(key1_loosen == 1)
				{
					key1_status = standby;
					HAL_GPIO_WritePin(ESC_POW_EN_GPIO_Port,ESC_POW_EN_Pin,GPIO_PIN_SET);
					runtimes = uwTick;
					key1_loosen = 0;
					HAL_Delay(3000);
					__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,100);
					__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,100);
					HAL_Delay(1000);
					
				}
			}
		}
		
		if(key1_status == standby)
		{
			if(key1_count >= 300)
			{
				if(key1_loosen == 1)
				{
					key1_status = stop;
					HAL_GPIO_WritePin(ESC_POW_EN_GPIO_Port,ESC_POW_EN_Pin,GPIO_PIN_RESET);
					key1_loosen = 0;
				}
			}
		}
		
		if(key1_status == on)
		{
			if(key1_loosen == 1)
			{
				if(key1_count >= 10)
				{
					key1_status = standby;
					key2_status = off;
					key1_loosen = 0;
				}
			}
		}
		
		if(key1_status == standby)
		{
			if(uwTick - runtimes >= 4*3600*1000)
			{
				key1_status = stop;
				HAL_GPIO_WritePin(ESC_POW_EN_GPIO_Port,ESC_POW_EN_Pin,GPIO_PIN_RESET);
			}
		}
		
		
		if(!HAL_GPIO_ReadPin(GPIOA,KEY2_Pin))
		{
			key2_count++;
		}
		else
		{
			key2_count = 0;
			key2_loosen = 1;
		}
		
		
		
		if(key1_status == standby)
		{
			if(key2_count >= 10)
			{
				if(key2_loosen == 1)
				{
					key1_status = on;
					key2_status = one;
					key2_loosen = 0;
				}
			}
		}
		else if(key2_status == one)
		{
			if(key2_count >= 10)
			{
				if(key2_loosen == 1)
				{
					key1_status = on;
					key2_status = two;
					key2_loosen = 0;
				}
			}
		}
		else if(key2_status == two)
		{
			if(key2_count >= 10)
			{
				if(key2_loosen == 1)
				{
					key1_status = on;
					key2_status = one;
					key2_loosen = 0;
				}
			}
		}
		else
		{
			
		}
		
		
		HAL_ADC_Start(&hadc);
		HAL_ADC_PollForConversion(&hadc,10);
		for(i=0;i<ARRAY_NUMBER;i++)
		{
			ADC_Value[i] = HAL_ADC_GetValue(&hadc);
		}
		Volt_Sum = 0;
		for(i=0;i<ARRAY_NUMBER;i++)
		{
			Volt_Sum += ADC_Value[i];
		}
		Volt_average = Volt_Sum/ARRAY_NUMBER;
		Volt_Real = (float)Volt_average/141.2244897959184f;
		
		
		if(key1_status == on)
		{
			if(key2_status == one)
			{
				pwm_target = 150;
			}
			else if(key2_status == two)
			{
				pwm_target = 175;
			}
		}
		else
		{
				pwm_target = 100;
		}
		
		if(pwm_temp < pwm_target)
		{
			pwm_temp += 0.3f;
		}
		else if(pwm_temp > pwm_target)
		{
			pwm_temp -= 1.0f;
		}
		else 
		{
			
		}
		pwm_temp = constrain(pwm_temp,100,200);
		
		if(Volt_Real >= 16.0f)
		{
			if(low_power_flag == 0)
			{
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,pwm_temp);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,pwm_temp);
			}
		}
		else 
		{
			key1_status = stop;
			key2_status = off;
			HAL_GPIO_WritePin(ESC_POW_EN_GPIO_Port,ESC_POW_EN_Pin,GPIO_PIN_RESET);
			low_power_flag = 1;
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,100);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,100);
		}
		
		if(low_power_flag)
		{
			if(Volt_Real >= 19.0f)
			{
				low_power_flag = 0;
			}
		}
		
		
		if((key1_count > 0)||(key2_count > 0))
		{
			__HAL_TIM_SET_COMPARE(&htim16,TIM_CHANNEL_1,20);
		}
		else
		{
			__HAL_TIM_SET_COMPARE(&htim16,TIM_CHANNEL_1,0);
		}

		
//		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,pwm_value);
//		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,pwm_value);
		
		
		
		HAL_Delay(10);
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 479;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 479;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 24;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ESC_POW_EN_GPIO_Port, ESC_POW_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ESC_POW_EN_Pin */
  GPIO_InitStruct.Pin = ESC_POW_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(ESC_POW_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY1_Pin KEY2_Pin */
  GPIO_InitStruct.Pin = KEY1_Pin|KEY2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
