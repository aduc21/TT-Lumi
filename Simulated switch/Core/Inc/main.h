/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define GREEN_1_Pin GPIO_PIN_0
#define GREEN_1_GPIO_Port GPIOA
#define RED1_Pin GPIO_PIN_1
#define RED1_GPIO_Port GPIOA
#define BLUE_1_Pin GPIO_PIN_3
#define BLUE_1_GPIO_Port GPIOA
#define B3_Pin GPIO_PIN_4
#define B3_GPIO_Port GPIOA
#define B4_Pin GPIO_PIN_0
#define B4_GPIO_Port GPIOB
#define RED_2_Pin GPIO_PIN_13
#define RED_2_GPIO_Port GPIOB
#define Buzzer_Pin GPIO_PIN_9
#define Buzzer_GPIO_Port GPIOC
#define BLUE_2_Pin GPIO_PIN_10
#define BLUE_2_GPIO_Port GPIOA
#define GREEN_2_Pin GPIO_PIN_11
#define GREEN_2_GPIO_Port GPIOA
#define B2_Pin GPIO_PIN_3
#define B2_GPIO_Port GPIOB
#define B5_Pin GPIO_PIN_4
#define B5_GPIO_Port GPIOB
#define B1_Pin GPIO_PIN_5
#define B1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
