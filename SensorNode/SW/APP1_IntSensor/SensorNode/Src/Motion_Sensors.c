/**
 ******************************************************************************
 * @file           : Int_Sensors.c
 * @brief          : Sensors Management
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


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "macro.h"
#include "cmsis_os.h"
#include "sai.h"
#include "gpio.h"
#include "Int_Sensors.h"
#include "Motion_Sensors.h"

/* Private includes ----------------------------------------------------------*/


/* Private typedef -----------------------------------------------------------*/

/* Number of conversions */

/* Private define ------------------------------------------------------------*/

#define MIC_BUFF_SIZE 	1600
#define MIC_BUFF_LEN 	MIC_BUFF_SIZE*2

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

uint16_t mic_buff[MIC_BUFF_LEN];
uint8_t k=0,y=0;
/* Private function prototypes -----------------------------------------------*/
extern void SystemClock_Config(void);

/* Private user code ---------------------------------------------------------*/

void MotionSensors_Task (void const * argument){

	uint8_t start = (uint8_t)((uint8_t *)argument);

	/* task disabled */
	if (!start){
		/*Configure GPIO pin Output Level */
		HAL_GPIO_WritePin(PW_SAI_GPIO_Port, PW_SAI_Pin, GPIO_PIN_RESET);
		GPIO_InitTypeDef GPIO_InitStruct = {0};
		GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_10;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		/*Configure GPIO pins : PCPin PCPin */
		GPIO_InitStruct.Pin = WU_AUDIO_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		HAL_GPIO_Init(WU_AUDIO_GPIO_Port, &GPIO_InitStruct);
		/* delete task */
		vTaskDelete( osThreadGetId () );
		return;
	}

	MX_SAI1_Init();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(PW_SAI_GPIO_Port, PW_SAI_Pin, GPIO_PIN_SET);
	osDelay(10);

	HAL_SAI_Receive_DMA(&hsai_BlockA1, (uint8_t *)mic_buff, MIC_BUFF_LEN);
	//HAL_SAI_Receive(&hsai_BlockA1, (uint8_t *)mic_buff, MIC_BUFF_LEN, 100);


	/* start application */
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,0);
	while (1){
		//HAL_SAI_Receive(&hsai_BlockA1, (uint8_t *)mic_buff, MIC_BUFF_LEN, 100);
		osDelay(1000);
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,0);
		//HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		//HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);
		/* exit stop mode */
		//SystemClock_Config();
	}

}




#pragma GCC push_options
#pragma GCC optimize ("O3")


void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai){
	k++;
}


void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai){
	y++;
}


#pragma GCC pop_options
