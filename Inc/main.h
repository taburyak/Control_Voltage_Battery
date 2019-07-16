/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#define BUTTON_RIGHT_Pin GPIO_PIN_1
#define BUTTON_RIGHT_GPIO_Port GPIOA
#define BUTTON_RIGHT_EXTI_IRQn EXTI1_IRQn
#define BUZZER_Pin GPIO_PIN_0
#define BUZZER_GPIO_Port GPIOB
#define LD_RED_Pin GPIO_PIN_1
#define LD_RED_GPIO_Port GPIOB
#define CS_SD_CARD_Pin GPIO_PIN_12
#define CS_SD_CARD_GPIO_Port GPIOB
#define BUTTON_LEFT_Pin GPIO_PIN_8
#define BUTTON_LEFT_GPIO_Port GPIOA
#define BUTTON_LEFT_EXTI_IRQn EXTI9_5_IRQn
#define BUTTON_UP_Pin GPIO_PIN_9
#define BUTTON_UP_GPIO_Port GPIOA
#define BUTTON_UP_EXTI_IRQn EXTI9_5_IRQn
#define BUTTON_DOWN_Pin GPIO_PIN_10
#define BUTTON_DOWN_GPIO_Port GPIOA
#define BUTTON_DOWN_EXTI_IRQn EXTI15_10_IRQn
#define BUTTON_SELECT_Pin GPIO_PIN_15
#define BUTTON_SELECT_GPIO_Port GPIOA
#define BUTTON_SELECT_EXTI_IRQn EXTI15_10_IRQn
#define LCD_BACKLIGHT_Pin GPIO_PIN_3
#define LCD_BACKLIGHT_GPIO_Port GPIOB
#define LCD_D4_Pin GPIO_PIN_4
#define LCD_D4_GPIO_Port GPIOB
#define LCD_D5_Pin GPIO_PIN_5
#define LCD_D5_GPIO_Port GPIOB
#define LCD_D6_Pin GPIO_PIN_6
#define LCD_D6_GPIO_Port GPIOB
#define LCD_D7_Pin GPIO_PIN_7
#define LCD_D7_GPIO_Port GPIOB
#define LCD_RS_Pin GPIO_PIN_8
#define LCD_RS_GPIO_Port GPIOB
#define LCD_E_Pin GPIO_PIN_9
#define LCD_E_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
