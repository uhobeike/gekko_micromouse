/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#define DEBUG_LED1_Pin GPIO_PIN_15
#define DEBUG_LED1_GPIO_Port GPIOC
#define DEBUG_LED3_Pin GPIO_PIN_0
#define DEBUG_LED3_GPIO_Port GPIOA
#define MotorL_Reverce_Pin GPIO_PIN_1
#define MotorL_Reverce_GPIO_Port GPIOA
#define MotorL_Foward_Pin GPIO_PIN_2
#define MotorL_Foward_GPIO_Port GPIOA
#define battery_watch_Pin GPIO_PIN_3
#define battery_watch_GPIO_Port GPIOA
#define sens_RR_Pin GPIO_PIN_4
#define sens_RR_GPIO_Port GPIOA
#define sens_R_Pin GPIO_PIN_5
#define sens_R_GPIO_Port GPIOA
#define sens_L_Pin GPIO_PIN_6
#define sens_L_GPIO_Port GPIOA
#define sens_RRA7_Pin GPIO_PIN_7
#define sens_RRA7_GPIO_Port GPIOA
#define R_LED_PWM_Pin GPIO_PIN_0
#define R_LED_PWM_GPIO_Port GPIOB
#define DEBUG_LED6_Pin GPIO_PIN_1
#define DEBUG_LED6_GPIO_Port GPIOB
#define DEBUG_LED5_Pin GPIO_PIN_2
#define DEBUG_LED5_GPIO_Port GPIOB
#define DEBUG_LED4_Pin GPIO_PIN_10
#define DEBUG_LED4_GPIO_Port GPIOB
#define CS_IMU_Pin GPIO_PIN_12
#define CS_IMU_GPIO_Port GPIOB
#define SCK_IMU_Pin GPIO_PIN_13
#define SCK_IMU_GPIO_Port GPIOB
#define MISO_IMU_Pin GPIO_PIN_14
#define MISO_IMU_GPIO_Port GPIOB
#define MOSI_IMU_Pin GPIO_PIN_15
#define MOSI_IMU_GPIO_Port GPIOB
#define USART_TX_Pin GPIO_PIN_9
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_10
#define USART_RX_GPIO_Port GPIOA
#define EncL_2_EXTI_Pin GPIO_PIN_15
#define EncL_2_EXTI_GPIO_Port GPIOA
#define EncL_1_EXTI_Pin GPIO_PIN_3
#define EncL_1_EXTI_GPIO_Port GPIOB
#define EncR_2_EXTI_Pin GPIO_PIN_4
#define EncR_2_EXTI_GPIO_Port GPIOB
#define EncR_1_EXTI_Pin GPIO_PIN_5
#define EncR_1_EXTI_GPIO_Port GPIOB
#define MotorR_Foward_Pin GPIO_PIN_6
#define MotorR_Foward_GPIO_Port GPIOB
#define MotorR_Reverce_Pin GPIO_PIN_7
#define MotorR_Reverce_GPIO_Port GPIOB
#define battery_watch_CE_Pin GPIO_PIN_9
#define battery_watch_CE_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
