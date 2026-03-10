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
#define SR04_ECHO1_Pin GPIO_PIN_0
#define SR04_ECHO1_GPIO_Port GPIOA
#define SR04_ECHO2_Pin GPIO_PIN_1
#define SR04_ECHO2_GPIO_Port GPIOA
#define SR04_ECHO3_Pin GPIO_PIN_2
#define SR04_ECHO3_GPIO_Port GPIOA
#define SR04_ECHO4_Pin GPIO_PIN_3
#define SR04_ECHO4_GPIO_Port GPIOA
#define SPI1_SCK_Pin GPIO_PIN_5
#define SPI1_SCK_GPIO_Port GPIOA
#define MOTOR1_PWM_Pin GPIO_PIN_6
#define MOTOR1_PWM_GPIO_Port GPIOA
#define MOTOR2_PWM_Pin GPIO_PIN_7
#define MOTOR2_PWM_GPIO_Port GPIOA
#define MOTOR3_PWM_Pin GPIO_PIN_0
#define MOTOR3_PWM_GPIO_Port GPIOB
#define ENC_A_Pin GPIO_PIN_9
#define ENC_A_GPIO_Port GPIOE
#define ENC_B_Pin GPIO_PIN_11
#define ENC_B_GPIO_Port GPIOE
#define MOTOR4_PWM_Pin GPIO_PIN_12
#define MOTOR4_PWM_GPIO_Port GPIOD
#define MOTOR5_PWM_Pin GPIO_PIN_13
#define MOTOR5_PWM_GPIO_Port GPIOD
#define MOTOR6_PWM_Pin GPIO_PIN_14
#define MOTOR6_PWM_GPIO_Port GPIOD
#define ESP_RX_Pin GPIO_PIN_9
#define ESP_RX_GPIO_Port GPIOA
#define ESP_TX_Pin GPIO_PIN_10
#define ESP_TX_GPIO_Port GPIOA
#define IR1_EXTI0_Pin GPIO_PIN_0
#define IR1_EXTI0_GPIO_Port GPIOD
#define IR2_EXTI1_Pin GPIO_PIN_1
#define IR2_EXTI1_GPIO_Port GPIOD
#define IR3_EXTI3_Pin GPIO_PIN_3
#define IR3_EXTI3_GPIO_Port GPIOD
#define IR4_EXTI4_Pin GPIO_PIN_4
#define IR4_EXTI4_GPIO_Port GPIOD
#define SPI1_MISO_Pin GPIO_PIN_4
#define SPI1_MISO_GPIO_Port GPIOB
#define SPI1_MOSI_Pin GPIO_PIN_5
#define SPI1_MOSI_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
