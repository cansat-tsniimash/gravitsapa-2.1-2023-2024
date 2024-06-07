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
#define Latch_Pin GPIO_PIN_1
#define Latch_GPIO_Port GPIOC
#define DS_1wire_Pin GPIO_PIN_1
#define DS_1wire_GPIO_Port GPIOA
#define SIRENA_Pin GPIO_PIN_2
#define SIRENA_GPIO_Port GPIOA
#define latch_diods_Pin GPIO_PIN_4
#define latch_diods_GPIO_Port GPIOA
#define Latch_RF_Pin GPIO_PIN_4
#define Latch_RF_GPIO_Port GPIOC
#define OE_RF_Pin GPIO_PIN_5
#define OE_RF_GPIO_Port GPIOC
#define SR_RST_Pin GPIO_PIN_0
#define SR_RST_GPIO_Port GPIOB
#define RED_BUTTON_Pin GPIO_PIN_1
#define RED_BUTTON_GPIO_Port GPIOB
#define GPS_TX_Pin GPIO_PIN_6
#define GPS_TX_GPIO_Port GPIOC
#define GPS_RX_Pin GPIO_PIN_7
#define GPS_RX_GPIO_Port GPIOC
#define diod_Pin GPIO_PIN_4
#define diod_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
