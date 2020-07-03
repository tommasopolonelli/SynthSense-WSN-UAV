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
#include "i2c.h"
#include "gpio.h"
#include "Int_Sensors.h"
#include "instance.h"

#include "iis2mdc_reg.h"
#include "bmp280.h"
#include "TMP117.h"
#include "HDC2080.h"
#include "lsm6dsox_reg.h"

#include "string.h"

/* Private includes ----------------------------------------------------------*/


/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/

enum {
  EVT_BMP280  						= (1 << 0), /* 0b0000000000000001 */
  EVT_TMP117               			= (1 << 1), /* 0b0000000000000010 */
  EVT_HDC2080             			= (1 << 2), /* 0b0000000000000100 */
  EVT_IIS2MDC             			= (1 << 3), /* 0b0000000000001000 */
  EVT_LSM6DSOX                 		= (1 << 4), /* 0b0000000000010000 */
  kCGDisplayRemoveFlag              = (1 << 5), /* 0b0000000000100000 */
  kCGDisplayEnabledFlag             = (1 << 8), /* 0b0000000100000000 */
  kCGDisplayDisabledFlag            = (1 << 9), /* 0b0000001000000000 */
  kCGDisplayMirrorFlag              = (1 << 10),/* 0b0000010000000000 */
  kCGDisplayUnMirrorFlag            = (1 << 11),/* 0b0000100000000000 */
  kCGDisplayDesktopShapeChangedFlag = (1 << 12) /* 0b0001000000000000 */
};

#define LOW_TIMER_MS 				(1000)
/* Private variables ---------------------------------------------------------*/

static StaticEventGroup_t IntSensors_EventGroupBuffer;
EventGroupHandle_t IntSensors_Event;

// An array to hold handles to the created timers.
static xTimerHandle Low_Timer;
static StaticTimer_t Low_Timer_Buffer;

//float temp, hum;
//Sensors_data_t Sen_dt;
Sensors_data_t Sen_dt_b[SENSOR_BUFF_SIZE];
uint16_t Sen_dt_p = 0;
uint16_t Sen_dt_stream = 0;

/* Private function prototypes -----------------------------------------------*/

static void Low_Timer_Callback( TimerHandle_t xTimer );
extern void SystemClock_Config(void);
/* Private user code ---------------------------------------------------------*/

void IntSensors_Task (void const * argument){

	uint8_t start = (uint8_t)((uint8_t *)argument);
	EventBits_t uxBits;
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* task disabled */
	if (!start){
		/*Configure GPIO pin Output Level */
		HAL_GPIO_WritePin(PW_INT_GPIO_Port, PW_INT_Pin, GPIO_PIN_RESET);
		/*Configure GPIO pin : PtPin */
		GPIO_InitStruct.Pin = TMP_IRQ_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		HAL_GPIO_Init(TMP_IRQ_GPIO_Port, &GPIO_InitStruct);
		/* Configure PINS I2C1 */
		GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		/* delete task */
		vTaskDelete( osThreadGetId () );
		return;
	}

	/* Disable deep sleep */
	Mngm_DeepSleep_en (0);

	/* Init HW */
	MX_I2C1_Init();

	 /*Configure GPIO pin Output Level */
	 HAL_GPIO_WritePin(PW_INT_GPIO_Port, PW_INT_Pin, GPIO_PIN_SET);
	 osDelay(100);

	// Create the event group
	IntSensors_Event = xEventGroupCreateStatic( &IntSensors_EventGroupBuffer );

	// Create low frequency timer
	Low_Timer = xTimerCreateStatic("Low_Timer", // Just a text name, not used by the kernel.
			LOW_TIMER_MS ,     					// The timer period in ticks.
			pdTRUE,         					// The timers will auto-reload themselves when they expire.
			"Low_Timer",     					// Assign each timer a unique id equal to its array index.
			Low_Timer_Callback,     			// Each timer calls the same callback when it expires.
			&Low_Timer_Buffer
	);

	/* Start Init Sensors */

	if (Init_bmp280(&hi2c1)){
		return;
	}
	osDelay(10);

	if (Init_TMP117(NULL)){
		return;
	}
	osDelay(10);

	Init_HDC2080();
	osDelay(10);

	if (Init_IIS2MDC()){
		return;
	}
	osDelay(10);

	if (Init_lsm6dsox()){
		return;
	}
	osDelay(10);

	/* start application */
	//xTimerStart( Low_Timer, 1000 );
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,0);
	while (1){

		/* Enable deep sleep */
		Mngm_DeepSleep_en (1);

		uxBits = xEventGroupWaitBits(
				/* The event group being tested. */
				IntSensors_Event,
				/* The bits within the event group to wait for. */
				EVT_BMP280 | EVT_TMP117 | EVT_HDC2080 | EVT_IIS2MDC | EVT_LSM6DSOX,
				/* BIT_0 & BIT_4 should be cleared before returning. */
				pdTRUE,
				/* Don't wait for both bits, either bit will do. */
				pdFALSE,
				/* Wait a maximum of 100ms for either bit to be set. */
				portMAX_DELAY );

		/* Disable deep sleep */
		Mngm_DeepSleep_en (0);

		if(IS_EVT(EVT_BMP280)){
			Sen_dt_b[Sen_dt_p].pres = Read_bmp280();
		}
		if(IS_EVT(EVT_TMP117)){
			Sen_dt_b[Sen_dt_p].temp = (float)TMP117_getTemperature();
		}
		if(IS_EVT(EVT_HDC2080)){
			Read_HDC2080(&Sen_dt_b[Sen_dt_p].temp_hum, &Sen_dt_b[Sen_dt_p].hum);
		}
		if(IS_EVT(EVT_IIS2MDC)){
			Read_IIS2MDC(Sen_dt_b[Sen_dt_p].magnetic_mG);
			/* only test */
			Read_lsm6dsox (Sen_dt_b[Sen_dt_p].acceleration, Sen_dt_b[Sen_dt_p].angular);
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			/* output buffer */
			Sen_dt_p++;
			if (Sen_dt_p == SENSOR_BUFF_SIZE){
				Sen_dt_p = 0;
				memset(Sen_dt_b,0,sizeof(Sen_dt_b));
			}
		}
		if(IS_EVT(EVT_LSM6DSOX)){
			//Read_lsm6dsox (Sen_dt_b[Sen_dt_p].acceleration, Sen_dt_b[Sen_dt_p].angular);
		}

		CHECK_STACK_OF();

	}


}


uint16_t Int_Sensors_Stream (uint8_t *out, uint16_t size){

	uint32_t itsize, Sen;


	if(Sen_dt_p == 0){
		return 0;
	}

	if(Sen_dt_stream >= Sen_dt_p){
		Sen_dt_stream = 0;
		/* end of stream */
		return 0;
	}

	/* how many bytes? */
	Sen = ((uint32_t)(Sen_dt_p - Sen_dt_stream))*sizeof(Sensors_data_t);
	/* floor round */
	itsize = size / sizeof(Sensors_data_t);
	if (Sen <= size){
		memcpy_DMA ((uint32_t *)&Sen_dt_b[Sen_dt_stream].temp,
				(uint32_t *)out,Sen);
		//Sen_dt_stream = 0;
	}else{
		Sen = itsize * sizeof(Sensors_data_t);
		memcpy_DMA ((uint32_t *)&Sen_dt_b[Sen_dt_stream].temp,
						(uint32_t *)out,Sen);
	}

	Sen_dt_stream += itsize;

	/* not wait */
	//memcpy_DMA_poll();

	return (uint16_t)Sen;

}



#pragma GCC push_options
#pragma GCC optimize ("O3")

static void Low_Timer_Callback( TimerHandle_t xTimer ){

	EVENTSETBIT(IntSensors_Event,EVT_BMP280 | EVT_TMP117 | EVT_HDC2080);

}

void IIS2MDC_Callback( void ){

	EVENTSETBIT(IntSensors_Event,EVT_IIS2MDC);

}

void lsm6dsox_Callback( void ){

	EVENTSETBIT(IntSensors_Event,EVT_LSM6DSOX);

}

void TMP117_Callback( void ){

	EVENTSETBIT(IntSensors_Event,EVT_TMP117);
	EVENTSETBIT(IntSensors_Event,EVT_BMP280);
	EVENTSETBIT(IntSensors_Event,EVT_HDC2080);

}

void HDC2080_Callback( void ){

	EVENTSETBIT(IntSensors_Event,EVT_HDC2080);

}

#pragma GCC pop_options
