/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define STATUS_Pin GPIO_PIN_3
#define STATUS_GPIO_Port GPIOE
#define LORA_RST_Pin GPIO_PIN_4
#define LORA_RST_GPIO_Port GPIOE
#define GPS_RST_Pin GPIO_PIN_5
#define GPS_RST_GPIO_Port GPIOE
#define BAT_V_Pin GPIO_PIN_0
#define BAT_V_GPIO_Port GPIOC
#define P1_DET_Pin GPIO_PIN_1
#define P1_DET_GPIO_Port GPIOC
#define P2_DET_Pin GPIO_PIN_2
#define P2_DET_GPIO_Port GPIOC
#define STLINK_RX_Pin GPIO_PIN_0
#define STLINK_RX_GPIO_Port GPIOA
#define STLINK_TX_Pin GPIO_PIN_1
#define STLINK_TX_GPIO_Port GPIOA
#define CS_IMU1_Pin GPIO_PIN_4
#define CS_IMU1_GPIO_Port GPIOA
#define CS_IMU2_Pin GPIO_PIN_4
#define CS_IMU2_GPIO_Port GPIOC
#define PYRO1_Pin GPIO_PIN_0
#define PYRO1_GPIO_Port GPIOB
#define PYRO2_Pin GPIO_PIN_1
#define PYRO2_GPIO_Port GPIOB
#define BUZZ_Pin GPIO_PIN_2
#define BUZZ_GPIO_Port GPIOB
#define LED_R_Pin GPIO_PIN_8
#define LED_R_GPIO_Port GPIOE
#define SRV_1_Pin GPIO_PIN_9
#define SRV_1_GPIO_Port GPIOE
#define LED_G_Pin GPIO_PIN_10
#define LED_G_GPIO_Port GPIOE
#define SRV_2_Pin GPIO_PIN_11
#define SRV_2_GPIO_Port GPIOE
#define LED_B_Pin GPIO_PIN_12
#define LED_B_GPIO_Port GPIOE
#define SRV_3_Pin GPIO_PIN_13
#define SRV_3_GPIO_Port GPIOE
#define SRV_4_Pin GPIO_PIN_14
#define SRV_4_GPIO_Port GPIOE
#define LED_Y_Pin GPIO_PIN_15
#define LED_Y_GPIO_Port GPIOE
#define CS_LORA_Pin GPIO_PIN_12
#define CS_LORA_GPIO_Port GPIOB
#define CS_EXT_Pin GPIO_PIN_8
#define CS_EXT_GPIO_Port GPIOD
#define LORA_IO0_Pin GPIO_PIN_9
#define LORA_IO0_GPIO_Port GPIOD
#define LORA_IO0_EXTI_IRQn EXTI9_5_IRQn
#define LORA_IO1_Pin GPIO_PIN_10
#define LORA_IO1_GPIO_Port GPIOD
#define LORA_IO1_EXTI_IRQn EXTI15_10_IRQn
#define INT_A1_Pin GPIO_PIN_14
#define INT_A1_GPIO_Port GPIOD
#define INT_A1_EXTI_IRQn EXTI15_10_IRQn
#define INT_G1_Pin GPIO_PIN_15
#define INT_G1_GPIO_Port GPIOD
#define INT_G1_EXTI_IRQn EXTI15_10_IRQn
#define INT_A2_Pin GPIO_PIN_6
#define INT_A2_GPIO_Port GPIOC
#define INT_A2_EXTI_IRQn EXTI9_5_IRQn
#define INT_G2_Pin GPIO_PIN_7
#define INT_G2_GPIO_Port GPIOC
#define INT_G2_EXTI_IRQn EXTI9_5_IRQn
#define INT_MAG_Pin GPIO_PIN_11
#define INT_MAG_GPIO_Port GPIOA
#define INT_MAG_EXTI_IRQn EXTI15_10_IRQn
#define INT_BMP_Pin GPIO_PIN_12
#define INT_BMP_GPIO_Port GPIOA
#define INT_BMP_EXTI_IRQn EXTI15_10_IRQn
#define SDIO_DET_FAKE_Pin GPIO_PIN_10
#define SDIO_DET_FAKE_GPIO_Port GPIOC
#define GPS_NF_Pin GPIO_PIN_11
#define GPS_NF_GPIO_Port GPIOC
#define IO1_Pin GPIO_PIN_0
#define IO1_GPIO_Port GPIOD
#define IO2_Pin GPIO_PIN_1
#define IO2_GPIO_Port GPIOD
#define IO3_Pin GPIO_PIN_4
#define IO3_GPIO_Port GPIOD
#define IO4_Pin GPIO_PIN_5
#define IO4_GPIO_Port GPIOD
#define IO5_Pin GPIO_PIN_6
#define IO5_GPIO_Port GPIOD
#define IO6_Pin GPIO_PIN_7
#define IO6_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
