/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define SV1_Pin GPIO_PIN_0
#define SV1_GPIO_Port GPIOA
#define SV2_Pin GPIO_PIN_1
#define SV2_GPIO_Port GPIOA
#define SV3_Pin GPIO_PIN_2
#define SV3_GPIO_Port GPIOA
#define SV4_Pin GPIO_PIN_3
#define SV4_GPIO_Port GPIOA
#define SS_Pin GPIO_PIN_4
#define SS_GPIO_Port GPIOA
#define SCK_Pin GPIO_PIN_5
#define SCK_GPIO_Port GPIOA
#define MISO_Pin GPIO_PIN_6
#define MISO_GPIO_Port GPIOA
#define MOSI_Pin GPIO_PIN_7
#define MOSI_GPIO_Port GPIOA
#define KC1_Pin GPIO_PIN_0
#define KC1_GPIO_Port GPIOB
#define KC2_Pin GPIO_PIN_1
#define KC2_GPIO_Port GPIOB
#define Line5_Pin GPIO_PIN_12
#define Line5_GPIO_Port GPIOB
#define Line4_Pin GPIO_PIN_13
#define Line4_GPIO_Port GPIOB
#define Line3_Pin GPIO_PIN_14
#define Line3_GPIO_Port GPIOB
#define Line2_Pin GPIO_PIN_15
#define Line2_GPIO_Port GPIOB
#define Line1_Pin GPIO_PIN_8
#define Line1_GPIO_Port GPIOA
#define P_Pin GPIO_PIN_4
#define P_GPIO_Port GPIOB
#define DP_Pin GPIO_PIN_5
#define DP_GPIO_Port GPIOB
#define XP_Pin GPIO_PIN_6
#define XP_GPIO_Port GPIOB
#define XT_Pin GPIO_PIN_7
#define XT_GPIO_Port GPIOB
#define DT_Pin GPIO_PIN_8
#define DT_GPIO_Port GPIOB
#define T_Pin GPIO_PIN_9
#define T_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
