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
#include "stm32wbxx_hal.h"

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
extern void GLOB_SET_EVT( void* xEventGroup, uint32_t x );
extern void Mngm_DeepSleep_en (uint8_t d);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define WU_AUDIO_Pin GPIO_PIN_13
#define WU_AUDIO_GPIO_Port GPIOC
#define WU_AUDIO_EXTI_IRQn EXTI15_10_IRQn
#define HDC_IRQ_Pin GPIO_PIN_3
#define HDC_IRQ_GPIO_Port GPIOC
#define HDC_IRQ_EXTI_IRQn EXTI3_IRQn
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define PW_FLASH_Pin GPIO_PIN_9
#define PW_FLASH_GPIO_Port GPIOA
#define IIS_IRQ_Pin GPIO_PIN_4
#define IIS_IRQ_GPIO_Port GPIOC
#define IIS_IRQ_EXTI_IRQn EXTI4_IRQn
#define LSM_IRQ2_Pin GPIO_PIN_5
#define LSM_IRQ2_GPIO_Port GPIOC
#define LSM_IRQ2_EXTI_IRQn EXTI9_5_IRQn
#define LSM_IRQ1_Pin GPIO_PIN_2
#define LSM_IRQ1_GPIO_Port GPIOB
#define LSM_IRQ1_EXTI_IRQn EXTI2_IRQn
#define DW_IRQ_Pin GPIO_PIN_0
#define DW_IRQ_GPIO_Port GPIOB
#define DW_IRQ_EXTI_IRQn EXTI0_IRQn
#define TMP_IRQ_Pin GPIO_PIN_1
#define TMP_IRQ_GPIO_Port GPIOB
#define TMP_IRQ_EXTI_IRQn EXTI1_IRQn
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_6
#define LED_GPIO_Port GPIOC
#define DW_RST_Pin GPIO_PIN_10
#define DW_RST_GPIO_Port GPIOC
#define DW_WU_Pin GPIO_PIN_11
#define DW_WU_GPIO_Port GPIOC
#define PW_EXT_Pin GPIO_PIN_0
#define PW_EXT_GPIO_Port GPIOD
#define PW_INT_Pin GPIO_PIN_1
#define PW_INT_GPIO_Port GPIOD
#define PW_SAI_Pin GPIO_PIN_5
#define PW_SAI_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
