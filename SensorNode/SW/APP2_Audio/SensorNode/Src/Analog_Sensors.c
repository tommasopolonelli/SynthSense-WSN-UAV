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
#include "adc.h"
#include "gpio.h"
#include "Int_Sensors.h"
#include "Analog_Sensors.h"

/* Private includes ----------------------------------------------------------*/


/* Private typedef -----------------------------------------------------------*/

/* Number of conversions */
#define NOC	(4)
#define __VREFANALOG_VOLTAGE__	(3300)

/* Private define ------------------------------------------------------------*/

static uint16_t raw_an_data[NOC*2];

/* Private macro -------------------------------------------------------------*/


#define LOW_TIMER_MS 				(1000)
/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/


/* Private user code ---------------------------------------------------------*/

void AnalogSensors_Task (void const * argument){

	uint8_t start = (uint8_t)((uint8_t *)argument);

	/* task disabled */
	if (!start){
		vTaskDelete( osThreadGetId () );
		return;
	}

	/* Init HW */
	MX_ADC1_Init();

	osDelay(10);

	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)raw_an_data, NOC*2);

	/* start application */
	while (1){
		osDelay(1000);
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,0);
	}

}




#pragma GCC push_options
#pragma GCC optimize ("O3")

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){

	uint16_t d;

	/* V in mV */
	d = __HAL_ADC_CALC_DATA_TO_VOLTAGE(__VREFANALOG_VOLTAGE__,raw_an_data[NOC],ADC_RESOLUTION_12B);
	/* V */
	Sen_dt.adc0 = ((float)d) / 1000;

	/* V in mV */
	d = __HAL_ADC_CALC_DATA_TO_VOLTAGE(__VREFANALOG_VOLTAGE__,raw_an_data[NOC+1],ADC_RESOLUTION_12B);
	/* V */
	Sen_dt.adc1 = ((float)d) / 1000;

	/* V in mV */
	d = __HAL_ADC_CALC_DATA_TO_VOLTAGE(__VREFANALOG_VOLTAGE__,raw_an_data[NOC+2],ADC_RESOLUTION_12B);
	/* V */
	Sen_dt.adc2 = ((float)d) / 1000;

	/* V in mV */
	d = __HAL_ADC_CALC_DATA_TO_VOLTAGE(__VREFANALOG_VOLTAGE__,raw_an_data[NOC+3],ADC_RESOLUTION_12B);
	/* V */
	Sen_dt.adc3 = ((float)d) / 1000;

}
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc){

	uint16_t d;

	/* V in mV */
	d = __HAL_ADC_CALC_DATA_TO_VOLTAGE(__VREFANALOG_VOLTAGE__,raw_an_data[0],ADC_RESOLUTION_12B);
	/* V */
	Sen_dt.adc0 = ((float)d) / 1000;

	/* V in mV */
	d = __HAL_ADC_CALC_DATA_TO_VOLTAGE(__VREFANALOG_VOLTAGE__,raw_an_data[1],ADC_RESOLUTION_12B);
	/* V */
	Sen_dt.adc1 = ((float)d) / 1000;

	/* V in mV */
	d = __HAL_ADC_CALC_DATA_TO_VOLTAGE(__VREFANALOG_VOLTAGE__,raw_an_data[2],ADC_RESOLUTION_12B);
	/* V */
	Sen_dt.adc2 = ((float)d) / 1000;

	/* V in mV */
	d = __HAL_ADC_CALC_DATA_TO_VOLTAGE(__VREFANALOG_VOLTAGE__,raw_an_data[3],ADC_RESOLUTION_12B);
	/* V */
	Sen_dt.adc3 = ((float)d) / 1000;

}


#pragma GCC pop_options
