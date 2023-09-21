/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "led7seg.h"
#include "ds18b20.h"
#include "DHT.h"
#include "keypad.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {ds_temp, dht_temp, dht_humi} status_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
DS18B20_Name DS1;
DHT_DataTypedef DHT11_Data;
float temp_ds, temp_dht, humi_dht;
status_t getting = ds_temp;
int period = 10; //set up period
float warning_temp = 40; //set up warning_temp
float warning_humi = 70; //set up warning_humi
bool getting_period = 0;
bool getting_warning = 0;
bool getting_warning_humi = 0;
char set_up[4];// = {'\0', '\0', '\0'};
int index_key = 0;
char key;
int count_period = 0;
char uart_receive[3];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
  /* With GCC, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
int __io_putchar(int ch)
#else
int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the UART3 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);

  return ch;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void set_up_led(status_t getting)
{
	if(getting == ds_temp)
	{
		HAL_GPIO_WritePin(led_temp_ds_GPIO_Port, led_temp_ds_Pin, SET);
		HAL_GPIO_WritePin(led_temp_dht_GPIO_Port, led_temp_dht_Pin, RESET);
		HAL_GPIO_WritePin(led_humi_dht_GPIO_Port, led_humi_dht_Pin, RESET);
	}
	else if(getting == dht_temp)
	{
		HAL_GPIO_WritePin(led_temp_ds_GPIO_Port, led_temp_ds_Pin, RESET);
		HAL_GPIO_WritePin(led_temp_dht_GPIO_Port, led_temp_dht_Pin, SET);
		HAL_GPIO_WritePin(led_humi_dht_GPIO_Port, led_humi_dht_Pin, RESET);
	}
	else if(getting == dht_humi)
	{
		HAL_GPIO_WritePin(led_temp_ds_GPIO_Port, led_temp_ds_Pin, RESET);
		HAL_GPIO_WritePin(led_temp_dht_GPIO_Port, led_temp_dht_Pin, RESET);
		HAL_GPIO_WritePin(led_humi_dht_GPIO_Port, led_humi_dht_Pin, SET);
	}
	if((temp_ds > warning_temp) || (temp_dht > warning_temp) || (humi_dht > warning_humi))
	{
		HAL_GPIO_WritePin(led_warning_GPIO_Port, led_warning_Pin, SET);
	}
	else
	{
		HAL_GPIO_WritePin(led_warning_GPIO_Port, led_warning_Pin, RESET);
	}
}

void read_sensor_task(void)
{
//	printf("Tick:%ld\n", uwTick);
	printf("In read_sensor_task!\n");
	temp_ds = DS18B20_ReadTemp(&DS1);
	printf("Read DS12B20!\n");
	DHT_GetData(&DHT11_Data);
	temp_dht = DHT11_Data.Temperature;
	humi_dht = DHT11_Data.Humidity;
	set_up_led(getting);
	printf("Read DHT11!\n");
	printf("Value temp_ds: %.2f!\n", temp_ds);
	printf("Value temp_dht: %.2f!\n", temp_dht);
	printf("Value humi_dht: %.2f!\n", humi_dht);
//	printf("Value temp_ds:%d!\n", (int) temp_ds);  //hien thi so nguyen
//	printf("Value temp_dht:%d!\n", (int) temp_dht);
//	printf("Value humi_dht:%d!\n", (int) humi_dht);
}

void key_pad_task(void)
{
	key = getKey();
	if(key)
	{
	  printf("Key: %c\n", key);
	  if(key == '*')
	  {
		  printf("Change period....\n");
		  getting_period = 1;
		  TIM4->CNT &= 0x0;
		  HAL_TIM_Base_Start_IT(&htim4);
		  index_key = 0;
		  key = 0;
	  }

	  else if(key == '#')
	  {
		  printf("Change warning temperature....\n");
		  getting_warning = 1;
		  getting_period = 0;
		  TIM4->CNT &= 0x0;
		  HAL_TIM_Base_Start_IT(&htim4);
		  index_key = 0;
		  key = 0;
	  }
	  else if((key == '0') && (getting_period == 0) && (getting_warning == 0) && (getting_warning_humi == 0))
	  {
		  printf("Change warning humidity....\n");
		  getting_warning_humi = 1;
		  TIM4->CNT &= 0x0;
		  HAL_TIM_Base_Start_IT(&htim4);
		  index_key = 0;
		  key = 0;
	  }
	  else if(getting_period)
	  {
		  set_up[index_key] = key;
		  index_key ++;
		  if(index_key > 1)
		  {
			  period = atoi(set_up);
			  printf("Changed period: %d\n", period);
			  HAL_TIM_Base_Stop_IT(&htim4);
			  getting_period = 0;
			  index_key = 0;
		  }
	  }
	  else if(getting_warning)
	  {
		  set_up[index_key] = key;
		  index_key ++;
		  if(index_key > 1)
		  {
			  warning_temp = (float) atoi(set_up);
			  printf("Changed warning_temp: %d\n", (int) warning_temp);
			  HAL_TIM_Base_Stop_IT(&htim4);
			  getting_warning = 0;
			  index_key = 0;
		  }
	  }
	  else if(getting_warning_humi)
	  {
		  set_up[index_key] = key;
		  index_key ++;
		  if(index_key > 1)
		  {
			  warning_humi = (float) atoi(set_up);
			  printf("Changed warning_humi: %d\n", (int) warning_humi);
			  HAL_TIM_Base_Stop_IT(&htim4);
			  getting_warning_humi = 0;
			  index_key = 0;
		  }
	  }
	  set_up_led(getting);
	  key = 0;
	}
}

void display_led7seg_task(void)
{
	if(getting == ds_temp)
	{
	  display_float(temp_ds);
	}
	else if(getting == dht_temp)
	{
	  display_float(temp_dht);
	}
	else if(getting == dht_humi)
	{
	  display_float(humi_dht);
	}
}

void uart_received_task(void)
{
	printf("Receive: %s\n", uart_receive);
		if(uart_receive[0] == '*')
		{
			char get_uart[2];
			strncpy(get_uart, uart_receive+1, 2);
			printf("Get uart to change period: %s\n", get_uart);
			period = atoi(get_uart);
			printf("Changed period: %d\n", period);
		}
		else if(uart_receive[0] == '#')
		{
			char get_uart[2];
			strncpy(get_uart, uart_receive+1, 2);
			printf("Get uart to change level warning_temp: %s\n", get_uart);
			warning_temp = (float) atoi(get_uart);
			printf("Changed warning_temp: %d\n", (int) warning_temp);
		}
		else if(uart_receive[0] == '0')
		{
			char get_uart[2];
			strncpy(get_uart, uart_receive+1, 2);
			printf("Get uart to change level warning_humi: %s\n", get_uart);
			warning_humi = (float) atoi(get_uart);
			printf("Changed warning_humi: %d\n", (int) warning_humi);
		}
		else if(strcmp(uart_receive, "md1") == 0)
		{
			getting = ds_temp;
			printf("Changed mode to ds18b20 temperature!\n");
		}
		else if(strcmp(uart_receive, "md2") == 0)
		{
			getting = dht_temp;
			printf("Changed mode to dht11 temperature!\n");
		}
		else if(strcmp(uart_receive, "md3") == 0)
		{
			getting = dht_humi;
			printf("Changed mode to dht11 humidity!\n");
		}
		else
		{
			for(int i = 0; i < 3; i ++)
			{
				uart_receive[i] = '\0';
			}
		}
		set_up_led(getting);
		HAL_UART_Receive_IT(&huart2, (uint8_t *) uart_receive, 3);
}

void button_set_mode_task(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == but_temp_ds_Pin)
	{
		if(getting != ds_temp)
		{
			getting = ds_temp;
			printf("Changed mode to ds18b20 temperature!\n");
		}
	}
	else if(GPIO_Pin == but_temp_dht_Pin)
	{
		if(getting != dht_temp)
		{
			getting = dht_temp;
			printf("Changed mode to dht11 temperature!\n");
		}
	}
	else if(GPIO_Pin == but_humi_dht_Pin)
	{
		if(getting != dht_humi)
		{
			getting = dht_humi;
			printf("Changed mode to dht11 humidity!\n");
		}
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	DS18B20_Init(&DS1, &htim2, ds18b20_GPIO_Port, ds18b20_Pin);
	HAL_Delay(2000);
	printf("Hello World!\n");
	HAL_UART_Receive_IT(&huart2, (uint8_t *) uart_receive, 3);
	TIM3->EGR = TIM_EGR_UG;     /* Force update for prescaler value. */
	TIM3->SR  = 0;              /* Clear update flag. */
	TIM4->EGR = TIM_EGR_UG;     /* Force update for prescaler value. */
	TIM4->SR  = 0;              /* Clear update flag. */
	HAL_TIM_Base_Start_IT(&htim3);
	read_sensor_task();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if((uwTick%10) == 1)
	  {
		  key_pad_task();
		  display_led7seg_task();
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7199;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7199;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 49999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, dht11_Pin|ds18b20_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ROW4_Pin|Digit1_pin_Pin|A_pin_Pin|F_pin_Pin
                          |Digit2_pin_Pin|Digit3_pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ROW3_Pin|ROW2_Pin|ROW1_Pin|led_humi_dht_Pin
                          |led_temp_dht_Pin|led_temp_ds_Pin|led_warning_Pin|B_pin_Pin
                          |Digit4_pin_Pin|G_pin_Pin|C_pin_Pin|Dot_pin_Pin
                          |D_pin_Pin|E_pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : but_temp_ds_Pin but_temp_dht_Pin but_humi_dht_Pin */
  GPIO_InitStruct.Pin = but_temp_ds_Pin|but_temp_dht_Pin|but_humi_dht_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : dht11_Pin ds18b20_Pin ROW4_Pin Digit1_pin_Pin
                           A_pin_Pin F_pin_Pin Digit2_pin_Pin Digit3_pin_Pin */
  GPIO_InitStruct.Pin = dht11_Pin|ds18b20_Pin|ROW4_Pin|Digit1_pin_Pin
                          |A_pin_Pin|F_pin_Pin|Digit2_pin_Pin|Digit3_pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : COLUMN3_Pin COLUMN2_Pin COLUMN1_Pin */
  GPIO_InitStruct.Pin = COLUMN3_Pin|COLUMN2_Pin|COLUMN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ROW3_Pin ROW2_Pin ROW1_Pin led_humi_dht_Pin
                           led_temp_dht_Pin led_temp_ds_Pin led_warning_Pin B_pin_Pin
                           Digit4_pin_Pin G_pin_Pin C_pin_Pin Dot_pin_Pin
                           D_pin_Pin E_pin_Pin */
  GPIO_InitStruct.Pin = ROW3_Pin|ROW2_Pin|ROW1_Pin|led_humi_dht_Pin
                          |led_temp_dht_Pin|led_temp_ds_Pin|led_warning_Pin|B_pin_Pin
                          |Digit4_pin_Pin|G_pin_Pin|C_pin_Pin|Dot_pin_Pin
                          |D_pin_Pin|E_pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* Prevent unused argument(s) compilation warning */
		UNUSED(htim);
	/* NOTE : This function should not be modified, when the callback is needed,
			the HAL_TIM_PeriodElapsedCallback could be implemented in the user file
	*/
	if (htim->Instance == TIM3)
	{
		count_period++;
		if(count_period >= period)
		{
			count_period = 0;
			read_sensor_task();
		}
	}
	if (htim->Instance == TIM4)
	{
		printf("Out setting!\n");
		getting_period = 0;
		getting_warning = 0;
		getting_warning_humi = 0;
		HAL_TIM_Base_Stop_IT(&htim4);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	/* Prevent unused argument(s) compilation warning */
		UNUSED(GPIO_Pin);
	/* NOTE: This function Should not be modified, when the callback is needed,
		   the HAL_GPIO_EXTI_Callback could be implemented in the user file
	*/
	button_set_mode_task(GPIO_Pin);
	set_up_led(getting);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	/* Prevent unused argument(s) compilation warning */
		UNUSED(huart);
	/* NOTE: This function should not be modified, when the callback is needed,
		   the HAL_UART_RxCpltCallback could be implemented in the user file
	*/
	uart_received_task();
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
