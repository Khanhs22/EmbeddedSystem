/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define but_temp_ds_Pin GPIO_PIN_13
#define but_temp_ds_GPIO_Port GPIOC
#define but_temp_dht_Pin GPIO_PIN_14
#define but_temp_dht_GPIO_Port GPIOC
#define but_humi_dht_Pin GPIO_PIN_15
#define but_humi_dht_GPIO_Port GPIOC
#define dht11_Pin GPIO_PIN_0
#define dht11_GPIO_Port GPIOA
#define ds18b20_Pin GPIO_PIN_1
#define ds18b20_GPIO_Port GPIOA
#define COLUMN3_Pin GPIO_PIN_4
#define COLUMN3_GPIO_Port GPIOA
#define COLUMN2_Pin GPIO_PIN_5
#define COLUMN2_GPIO_Port GPIOA
#define COLUMN1_Pin GPIO_PIN_6
#define COLUMN1_GPIO_Port GPIOA
#define ROW4_Pin GPIO_PIN_7
#define ROW4_GPIO_Port GPIOA
#define ROW3_Pin GPIO_PIN_0
#define ROW3_GPIO_Port GPIOB
#define ROW2_Pin GPIO_PIN_1
#define ROW2_GPIO_Port GPIOB
#define ROW1_Pin GPIO_PIN_10
#define ROW1_GPIO_Port GPIOB
#define led_humi_dht_Pin GPIO_PIN_12
#define led_humi_dht_GPIO_Port GPIOB
#define led_temp_dht_Pin GPIO_PIN_13
#define led_temp_dht_GPIO_Port GPIOB
#define led_temp_ds_Pin GPIO_PIN_14
#define led_temp_ds_GPIO_Port GPIOB
#define led_warning_Pin GPIO_PIN_15
#define led_warning_GPIO_Port GPIOB
#define Digit1_pin_Pin GPIO_PIN_9
#define Digit1_pin_GPIO_Port GPIOA
#define A_pin_Pin GPIO_PIN_10
#define A_pin_GPIO_Port GPIOA
#define F_pin_Pin GPIO_PIN_11
#define F_pin_GPIO_Port GPIOA
#define Digit2_pin_Pin GPIO_PIN_12
#define Digit2_pin_GPIO_Port GPIOA
#define Digit3_pin_Pin GPIO_PIN_15
#define Digit3_pin_GPIO_Port GPIOA
#define B_pin_Pin GPIO_PIN_3
#define B_pin_GPIO_Port GPIOB
#define Digit4_pin_Pin GPIO_PIN_4
#define Digit4_pin_GPIO_Port GPIOB
#define G_pin_Pin GPIO_PIN_5
#define G_pin_GPIO_Port GPIOB
#define C_pin_Pin GPIO_PIN_6
#define C_pin_GPIO_Port GPIOB
#define Dot_pin_Pin GPIO_PIN_7
#define Dot_pin_GPIO_Port GPIOB
#define D_pin_Pin GPIO_PIN_8
#define D_pin_GPIO_Port GPIOB
#define E_pin_Pin GPIO_PIN_9
#define E_pin_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
