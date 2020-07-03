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
#include "pdm2pcm_glo.h"
#include "Flash_Mngm.h"

/* Private includes ----------------------------------------------------------*/


/* Private typedef -----------------------------------------------------------*/

/* Number of conversions */

/* Private define ------------------------------------------------------------*/

/* Rec timer after the last trigger */
#define REC_TIMER_MS 				(5000)


/* The high-pass filter is a one-pole recursive filter. The cut-off frequency is configured by
modifying the parameter high_pass_tap from the PDM_Filter_Handler_t. This coefficient
value must be in the range [0 : 1]. The format used is Q0.31, meaning that 1 corresponds to
the maximum integer value obtainable with 31-bit resolution. For example, configuring the
high_pass_tap parameter to 0.98 corresponds to 0.98*(231-1) = 2104533974.
 */
#define HIGH_PASS_FILT				(2122358088)
#define PDM_FILTER_NO_ERROR			(0)

#define	DEFAULT_MIC_GAIN			(15)

/* chanche both numbers */
#define DEC_FAC						PDM_FILTER_DEC_FACTOR_128
#define DEC_FAC_NUM					(128)

/* number of samples generated at every filter operations */
/* must be larger as possible to reduce the load */
/* increasing the size increases the RAM size */
#define PCM_SAMPLES					128
#define MIC_BUFF_TYPE				uint16_t

#define MIC_BUFF_SIZE 				((DEC_FAC_NUM * PCM_SAMPLES) / (8*sizeof(MIC_BUFF_TYPE)))
#define MIC_BUFF_LEN 				(MIC_BUFF_SIZE*2)

/* Private macro -------------------------------------------------------------*/

enum {
	EVT_RECHALF  				      = (1 << 0), /* 0b0000000000000001 */
	EVT_RECFULL               	      = (1 << 1), /* 0b0000000000000010 */
	EVT_MOTIONSTART             	  = (1 << 2), /* 0b0000000000000100 */
	EVT_MOTIONEND             		  = (1 << 3), /* 0b0000000000001000 */
	EVT_MOTIONSEND                 	  = (1 << 4), /* 0b0000000000010000 */
	kCGDisplayRemoveFlag              = (1 << 5), /* 0b0000000000100000 */
	kCGDisplayEnabledFlag             = (1 << 8), /* 0b0000000100000000 */
	kCGDisplayDisabledFlag            = (1 << 9), /* 0b0000001000000000 */
	kCGDisplayMirrorFlag              = (1 << 10),/* 0b0000010000000000 */
	kCGDisplayUnMirrorFlag            = (1 << 11),/* 0b0000100000000000 */
	kCGDisplayDesktopShapeChangedFlag = (1 << 12) /* 0b0001000000000000 */
};

typedef enum {
	M_S_SLEEP,
	M_S_START,
	M_S_REC,
	M_S_DATATX
}motion_stat_e;

/* Private variables ---------------------------------------------------------*/

MIC_BUFF_TYPE mic_buff[MIC_BUFF_LEN];
static int16_t pcm_buffer[PCM_SAMPLES];

static StaticEventGroup_t Motion_EventGroupBuffer;
EventGroupHandle_t Motion_Event;

// An array to hold handles to the created timers.
static xTimerHandle Rec_Timer;
static StaticTimer_t Rec_Timer_Buffer;

/* status */
motion_stat_e motion_stat = M_S_SLEEP;

/* PDM filter */
static PDM_Filter_Handler_t PDM1_filter_handler;
static PDM_Filter_Config_t PDM1_filter_config;

/* Private function prototypes -----------------------------------------------*/
static void Rec_Timer_Callback( TimerHandle_t xTimer );

/* Private user code ---------------------------------------------------------*/

void MotionSensors_Task (void const * argument){

	EventBits_t uxBits;

	uint8_t start = (uint8_t)((uint8_t *)argument);

	/* task disabled */
	if (!start){
		/*Configure GPIO pin Output Level */
		HAL_GPIO_WritePin(PW_SAI_GPIO_Port, PW_SAI_Pin, GPIO_PIN_RESET);
		/* delete task */
		vTaskDelete( osThreadGetId () );
		return;
	}

	MX_SAI1_Init();
	/* Set the sampling freq a 2048KHz */
	hsai_BlockA1.Init.Mckdiv = 2;
	hsai_BlockA1.Instance->CR1 |= (hsai_BlockA1.Init.Mckdiv << 20);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(PW_SAI_GPIO_Port, PW_SAI_Pin, GPIO_PIN_SET);
	osDelay(1000);

	// Create the event group
	Motion_Event = xEventGroupCreateStatic( &Motion_EventGroupBuffer );

	// Create low frequency timer
	Rec_Timer = xTimerCreateStatic("Rec_Timer", // Just a text name, not used by the kernel.
			REC_TIMER_MS ,     					// The timer period in ticks.
			pdFALSE,         					// The timers will auto-reload themselves when they expire.
			"Rec_Timer",     					// Assign each timer a unique id equal to its array index.
			Rec_Timer_Callback,     			// Each timer calls the same callback when it expires.
			&Rec_Timer_Buffer
	);

	//Init PDM Filter
	/* Initialize PDM Filter structure */
	PDM1_filter_handler.bit_order = PDM_FILTER_BIT_ORDER_LSB;
	PDM1_filter_handler.endianness = PDM_FILTER_ENDIANNESS_LE;
	PDM1_filter_handler.high_pass_tap = HIGH_PASS_FILT;
	PDM1_filter_handler.out_ptr_channels = 1;
	PDM1_filter_handler.in_ptr_channels = 1;
	if(PDM_Filter_Init((PDM_Filter_Handler_t *)(&PDM1_filter_handler)) != PDM_FILTER_NO_ERROR){
		while(1);
	}

	PDM1_filter_config.output_samples_number = PCM_SAMPLES;
	PDM1_filter_config.mic_gain = DEFAULT_MIC_GAIN;
	PDM1_filter_config.decimation_factor = DEC_FAC;
	if(PDM_Filter_setConfig((PDM_Filter_Handler_t *)&PDM1_filter_handler,
			&PDM1_filter_config) != PDM_FILTER_NO_ERROR){
		while(1);
	}

	/* start application */
	while (1){

		switch(motion_stat){
		case M_S_SLEEP :

			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,0);

			uxBits = xEventGroupWaitBits(
					/* The event group being tested. */
					Motion_Event,
					/* The bits within the event group to wait for. */
					EVT_MOTIONSTART | EVT_MOTIONSEND,
					/* BIT_0 & BIT_4 should be cleared before returning. */
					pdTRUE,
					/* Don't wait for both bits, either bit will do. */
					pdFALSE,
					/* Wait a maximum of 100ms for either bit to be set. */
					portMAX_DELAY );

			if(IS_EVT(EVT_MOTIONSTART)){
				HAL_SAI_Receive_DMA(&hsai_BlockA1, (uint8_t *)mic_buff, MIC_BUFF_LEN);
				xTimerStart( Rec_Timer, 1000 );
				motion_stat = M_S_REC;
			}

			if(IS_EVT(EVT_MOTIONSEND)){
				motion_stat = M_S_DATATX;
			}

			break;
		case M_S_START:
			break;
		case M_S_REC:
			uxBits = xEventGroupWaitBits(
					/* The event group being tested. */
					Motion_Event,
					/* The bits within the event group to wait for. */
					EVT_MOTIONSTART | EVT_RECHALF | EVT_RECFULL | EVT_MOTIONEND | EVT_MOTIONSEND,
					/* BIT_0 & BIT_4 should be cleared before returning. */
					pdTRUE,
					/* Don't wait for both bits, either bit will do. */
					pdFALSE,
					/* Wait a maximum of 100ms for either bit to be set. */
					portMAX_DELAY );

			if(IS_EVT(EVT_MOTIONSTART)){
				xTimerReset( Rec_Timer, 1000 );
			}
			if(IS_EVT(EVT_RECHALF)){
				HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

				/* process current frame */
				PDM_Filter(&mic_buff[0], &pcm_buffer[0], &PDM1_filter_handler);

				/* Store data to Flash */
				Flash_Write_Req ((uint8_t *)&pcm_buffer[0], sizeof(pcm_buffer));
			}
			if(IS_EVT(EVT_RECFULL)){

				/* process current frame */
				PDM_Filter(&mic_buff[MIC_BUFF_SIZE], &pcm_buffer[0], &PDM1_filter_handler);

				/* Store data to Flash */
				Flash_Write_Req ((uint8_t *)&pcm_buffer[0], sizeof(pcm_buffer));
			}
			if(IS_EVT(EVT_MOTIONEND)){
				HAL_SAI_DMAStop(&hsai_BlockA1);
				motion_stat = M_S_SLEEP;
			}
			if(IS_EVT(EVT_MOTIONSEND)){
				HAL_SAI_DMAStop(&hsai_BlockA1);
				xTimerStop( Rec_Timer, 1000 );
				motion_stat = M_S_DATATX;
			}
			break;
		case M_S_DATATX:

			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,0);

			uxBits = xEventGroupWaitBits(
					/* The event group being tested. */
					Motion_Event,
					/* The bits within the event group to wait for. */
					EVT_MOTIONSTART | EVT_RECHALF | EVT_RECFULL | EVT_MOTIONEND | EVT_MOTIONSEND,
					/* BIT_0 & BIT_4 should be cleared before returning. */
					pdTRUE,
					/* Don't wait for both bits, either bit will do. */
					pdFALSE,
					/* Wait a maximum of 100ms for either bit to be set. */
					portMAX_DELAY );

			if(IS_EVT(EVT_MOTIONSEND)){
				/* end of transfer */
				motion_stat = M_S_SLEEP;
			}

			break;

		}

		//HAL_SAI_Receive(&hsai_BlockA1, (uint8_t *)mic_buff, MIC_BUFF_LEN, 100);
		//osDelay(1000);
		//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,0);
		//HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		//HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);
		/* exit stop mode */
		//SystemClock_Config();
	}

}




#pragma GCC push_options
#pragma GCC optimize ("O3")


void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai){
	EVENTSETBIT(Motion_Event,EVT_RECHALF);
}


void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai){
	EVENTSETBIT(Motion_Event,EVT_RECFULL);
}

static void Rec_Timer_Callback( TimerHandle_t xTimer ){
	EVENTSETBIT(Motion_Event,EVT_MOTIONEND);
}

void WUA_Callback( void ){
	EVENTSETBIT(Motion_Event,EVT_MOTIONSTART);
}

void MotionSens_SendState( void ){
	EVENTSETBIT(Motion_Event,EVT_MOTIONSEND);
}


#pragma GCC pop_options
