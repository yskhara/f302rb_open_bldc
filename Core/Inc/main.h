/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32f3xx_hal.h"
#include "stm32f3xx_ll_adc.h"
#include "stm32f3xx_ll_dac.h"
#include "stm32f3xx_ll_dma.h"
#include "stm32f3xx_ll_rcc.h"
#include "stm32f3xx_ll_bus.h"
#include "stm32f3xx_ll_system.h"
#include "stm32f3xx_ll_exti.h"
#include "stm32f3xx_ll_cortex.h"
#include "stm32f3xx_ll_utils.h"
#include "stm32f3xx_ll_pwr.h"
#include "stm32f3xx_ll_gpio.h"

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
#define Q31(x) ((q31_t)((x) * 2147483648))
#define Q31_TO_F(x) ((float32_t)(x) / 2147483648)
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define Curr_fdbk_PhC_Pin GPIO_PIN_0
#define Curr_fdbk_PhC_GPIO_Port GPIOC
#define Curr_fdbk_PhB_Pin GPIO_PIN_1
#define Curr_fdbk_PhB_GPIO_Port GPIOC
#define Temperature_feedback_Pin GPIO_PIN_2
#define Temperature_feedback_GPIO_Port GPIOC
#define BEMF1_sensing_Pin GPIO_PIN_3
#define BEMF1_sensing_GPIO_Port GPIOC
#define Curr_fdbk_PhA_Pin GPIO_PIN_0
#define Curr_fdbk_PhA_GPIO_Port GPIOA
#define VBUS_Sensing_Pin GPIO_PIN_1
#define VBUS_Sensing_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define BEMF3_sensing_Pin GPIO_PIN_7
#define BEMF3_sensing_GPIO_Port GPIOA
#define BEMF2_sensing_Pin GPIO_PIN_0
#define BEMF2_sensing_GPIO_Port GPIOB
#define POTENTIOMETER_Pin GPIO_PIN_1
#define POTENTIOMETER_GPIO_Port GPIOB
#define LED_RED_Pin GPIO_PIN_2
#define LED_RED_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_13
#define LD2_GPIO_Port GPIOB
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
