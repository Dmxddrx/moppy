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
#define M5_AIN1_Pin GPIO_PIN_4
#define M5_AIN1_GPIO_Port GPIOE
#define M5_AIN2_Pin GPIO_PIN_5
#define M5_AIN2_GPIO_Port GPIOE
#define M6_BIN1_Pin GPIO_PIN_6
#define M6_BIN1_GPIO_Port GPIOE
#define M1_AIN2_Pin GPIO_PIN_0
#define M1_AIN2_GPIO_Port GPIOC
#define M1_AIN1_Pin GPIO_PIN_1
#define M1_AIN1_GPIO_Port GPIOC
#define BTN_OLED_PAGE_Pin GPIO_PIN_2
#define BTN_OLED_PAGE_GPIO_Port GPIOC
#define M1_PWM_Pin GPIO_PIN_6
#define M1_PWM_GPIO_Port GPIOA
#define M2_PWM_Pin GPIO_PIN_7
#define M2_PWM_GPIO_Port GPIOA
#define M2_BIN1_Pin GPIO_PIN_4
#define M2_BIN1_GPIO_Port GPIOC
#define M2_BIN2_Pin GPIO_PIN_5
#define M2_BIN2_GPIO_Port GPIOC
#define M3_PWM_Pin GPIO_PIN_0
#define M3_PWM_GPIO_Port GPIOB
#define TRIG2_Pin GPIO_PIN_1
#define TRIG2_GPIO_Port GPIOB
#define M6_BIN2_Pin GPIO_PIN_7
#define M6_BIN2_GPIO_Port GPIOE
#define TRIG3_Pin GPIO_PIN_8
#define TRIG3_GPIO_Port GPIOE
#define ENC1_A_Pin GPIO_PIN_9
#define ENC1_A_GPIO_Port GPIOE
#define OLED_SCL_Pin GPIO_PIN_10
#define OLED_SCL_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_11
#define OLED_SDA_GPIO_Port GPIOB
#define M2_STBY_Pin GPIO_PIN_8
#define M2_STBY_GPIO_Port GPIOD
#define M3_STBY_Pin GPIO_PIN_9
#define M3_STBY_GPIO_Port GPIOD
#define M4_PWM_Pin GPIO_PIN_12
#define M4_PWM_GPIO_Port GPIOD
#define M5_PWM_Pin GPIO_PIN_13
#define M5_PWM_GPIO_Port GPIOD
#define M6_PWM_Pin GPIO_PIN_14
#define M6_PWM_GPIO_Port GPIOD
#define ENC2_A_Pin GPIO_PIN_6
#define ENC2_A_GPIO_Port GPIOC
#define M4_BIN1_Pin GPIO_PIN_8
#define M4_BIN1_GPIO_Port GPIOC
#define M4_BIN2_Pin GPIO_PIN_9
#define M4_BIN2_GPIO_Port GPIOC
#define TRIG1_Pin GPIO_PIN_8
#define TRIG1_GPIO_Port GPIOA
#define ESP_RX_Pin GPIO_PIN_9
#define ESP_RX_GPIO_Port GPIOA
#define ESP_TX_Pin GPIO_PIN_10
#define ESP_TX_GPIO_Port GPIOA
#define IR1_EXTI0_Pin GPIO_PIN_0
#define IR1_EXTI0_GPIO_Port GPIOD
#define IR2_EXTI1_Pin GPIO_PIN_1
#define IR2_EXTI1_GPIO_Port GPIOD
#define M1_STBY_Pin GPIO_PIN_2
#define M1_STBY_GPIO_Port GPIOD
#define IR3_EXTI3_Pin GPIO_PIN_3
#define IR3_EXTI3_GPIO_Port GPIOD
#define IR4_EXTI4_Pin GPIO_PIN_4
#define IR4_EXTI4_GPIO_Port GPIOD
#define M3_AIN1_Pin GPIO_PIN_5
#define M3_AIN1_GPIO_Port GPIOD
#define M3_AIN2_Pin GPIO_PIN_6
#define M3_AIN2_GPIO_Port GPIOD
#define TRIG4_Pin GPIO_PIN_7
#define TRIG4_GPIO_Port GPIOD
#define SCL1_Pin GPIO_PIN_6
#define SCL1_GPIO_Port GPIOB
#define SDA1_Pin GPIO_PIN_7
#define SDA1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
