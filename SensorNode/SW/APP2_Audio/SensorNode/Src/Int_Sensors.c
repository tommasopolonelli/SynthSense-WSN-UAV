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
#include "iis2mdc_reg.h"

#include "bmp280.h"
#include "TMP117.h"
#include "HDC2080.h"
#include "lsm6dsox_reg.h"

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

float temp, hum;
Sensors_data_t Sen_dt;

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
		/* delete task */
		vTaskDelete( osThreadGetId () );
		return;
	}

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

		if(IS_EVT(EVT_BMP280)){
			Sen_dt.pres = Read_bmp280();
		}
		if(IS_EVT(EVT_TMP117)){
			Sen_dt.temp = (float)TMP117_getTemperature();
		}
		if(IS_EVT(EVT_HDC2080)){
			Read_HDC2080(&Sen_dt.temp_hum, &Sen_dt.hum);
		}
		if(IS_EVT(EVT_IIS2MDC)){
			Read_IIS2MDC(Sen_dt.magnetic_mG);
			/* only test */
			Read_lsm6dsox (Sen_dt.acceleration, Sen_dt.angular);
			//HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			//HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);
			//HAL_DBGMCU_EnableDBGStopMode();
			//HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
			/* exit stop mode */
			//SystemClock_Config();
		}
		if(IS_EVT(EVT_LSM6DSOX)){
			Read_lsm6dsox (Sen_dt.acceleration, Sen_dt.angular);
		}

		CHECK_STACK_OF();

	}


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

#pragma GCC pop_options
