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
#include "quadspi.h"
#include "gpio.h"
#include "Int_Sensors.h"
#include "Flash_Mngm.h"

/* Private includes ----------------------------------------------------------*/


/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

enum {
	EVT_WRITEREQ  					= (1 << 0), /* 0b0000000000000001 */
	EVT_READREQ               	    = (1 << 1), /* 0b0000000000000010 */
	EVT_NULL11             	    	= (1 << 2), /* 0b0000000000000100 */
	EVT_NULL12             			= (1 << 3), /* 0b0000000000001000 */
	EVT_NULL13                 		= (1 << 4), /* 0b0000000000010000 */
	kCGDisplayRemoveFlag              = (1 << 5), /* 0b0000000000100000 */
	kCGDisplayEnabledFlag             = (1 << 8), /* 0b0000000100000000 */
	kCGDisplayDisabledFlag            = (1 << 9), /* 0b0000001000000000 */
	kCGDisplayMirrorFlag              = (1 << 10),/* 0b0000010000000000 */
	kCGDisplayUnMirrorFlag            = (1 << 11),/* 0b0000100000000000 */
	kCGDisplayDesktopShapeChangedFlag = (1 << 12) /* 0b0001000000000000 */
};

/* N25Q064A Micron memory */
/* Size of the flash */
#define QSPI_FLASH_SIZE                      24  //address bit
#define QSPI_PAGE_SIZE                       256

/* Identification Operations */
#define READ_ID_CMD                          0x9E
#define READ_ID_CMD2                         0x9F
#define MULTIPLE_IO_READ_ID_CMD              0xAF
#define READ_SERIAL_FLASH_DISCO_PARAM_CMD    0x5A

/* Read Operations */
#define READ_CMD                             0x03
#define FAST_READ_CMD                        0x0B
#define DUAL_OUT_FAST_READ_CMD               0x3B
#define DUAL_INOUT_FAST_READ_CMD             0xBB
#define QUAD_OUT_FAST_READ_CMD               0x6B
#define QUAD_INOUT_FAST_READ_CMD             0xEB

/* Write Operations */
#define WRITE_ENABLE_CMD                     0x06
#define WRITE_DISABLE_CMD                    0x04

/* Register Operations */
#define READ_STATUS_REG_CMD                  0x05
#define WRITE_STATUS_REG_CMD                 0x01

#define READ_LOCK_REG_CMD                    0xE8
#define WRITE_LOCK_REG_CMD                   0xE5

#define READ_FLAG_STATUS_REG_CMD             0x70
#define CLEAR_FLAG_STATUS_REG_CMD            0x50

#define READ_NONVOL_CFG_REG_CMD              0xB5
#define WRITE_NONVOL_CFG_REG_CMD             0xB1

#define READ_VOL_CFG_REG_CMD                 0x85
#define WRITE_VOL_CFG_REG_CMD                0x81

#define READ_ENHANCED_VOL_CFG_REG_CMD        0x65
#define WRITE_ENHANCED_VOL_CFG_REG_CMD       0x61

/* Program Operations */
#define PAGE_PROG_CMD                        0x02
#define DUAL_IN_FAST_PROG_CMD                0xA2
#define EXT_DUAL_IN_FAST_PROG_CMD            0xD2
#define QUAD_IN_FAST_PROG_CMD                0x32
#define EXT_QUAD_IN_FAST_PROG_CMD            0x12

/* Erase Operations */
#define SUBSECTOR_ERASE_CMD                  0x20
#define SECTOR_ERASE_CMD                     0xD8
#define BULK_ERASE_CMD                       0xC7
#define PROG_ERASE_RESUME_CMD                0x7A
#define PROG_ERASE_SUSPEND_CMD               0x75

/* One-Time Programmable Operations */
#define READ_OTP_ARRAY_CMD                   0x4B
#define PROG_OTP_ARRAY_CMD                   0x42

/* Default dummy clocks cycles */
#define DUMMY_CLOCK_CYCLES_READ              2

/* End address of the QSPI memory */
#define QSPI_END_ADDR              			((1 << QSPI_FLASH_SIZE)-1)
#define CHECK_END_ADDR(x)					(x < QSPI_END_ADDR)

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Buffer used for transmission */
static volatile uint8_t CmdCplt, RxCplt, TxCplt, StatusMatch;
static QSPI_CommandTypeDef sCommand;

/* Event */
static StaticEventGroup_t Flash_EventGroupBuffer;
EventGroupHandle_t Flash_Event;

/* ext command */
static uint8_t *aTxBuffer;
static uint16_t aTxBuffer_size;

/* state */
static uint8_t step = 0;

/* output frame */
static uint32_t Sen_dt_stream = 0;
static uint32_t address = 0;

/* Private function prototypes -----------------------------------------------*/

static void QSPI_DummyCyclesCfg(QSPI_HandleTypeDef *hqspi);
static void QSPI_AutoPollingMemReady(QSPI_HandleTypeDef *hqspi);
static void QSPI_WriteEnable(QSPI_HandleTypeDef *hqspi);
/* Private user code ---------------------------------------------------------*/

void FlashMngm_Task (void const * argument){

	uint8_t start = (uint8_t)((uint8_t *)argument);

	EventBits_t uxBits;

	/* task disabled */
	if (!start){
		/*Configure GPIO pin Output Level */
		HAL_GPIO_WritePin(PW_FLASH_GPIO_Port, PW_FLASH_Pin, GPIO_PIN_RESET);
		/* delete task */
		vTaskDelete( osThreadGetId () );
		return;
	}

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(PW_FLASH_GPIO_Port, PW_FLASH_Pin, GPIO_PIN_SET);
	osDelay(10);

	MX_QUADSPI_Init();

	// Create the event group
	Flash_Event = xEventGroupCreateStatic( &Flash_EventGroupBuffer );

	/* start application */
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,0);

	/* USER CODE BEGIN WHILE */
	sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
	sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
	sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
	sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

	while (1)
	{
		switch(step)
		{
		case 0:
			CmdCplt = 0;

			/* Initialize Reception buffer --------------------------------------- */
			address = 0;

			/* Enable write operations ------------------------------------------- */
			QSPI_WriteEnable(&hqspi);

			/* Erasing Sequence -------------------------------------------------- */

			// Erase Sector */
			/*sCommand.Instruction = SUBSECTOR_ERASE_CMD;
		        sCommand.AddressMode = QSPI_ADDRESS_1_LINE;
		        sCommand.Address     = address;
		        sCommand.DataMode    = QSPI_DATA_NONE;
		        sCommand.DummyCycles = 0;*/

			/* Erase all */
			sCommand.Instruction = SUBSECTOR_ERASE_CMD;
			sCommand.Instruction = BULK_ERASE_CMD;  // Erase ALL
			sCommand.AddressMode = QSPI_ADDRESS_NONE;
			sCommand.DataMode    = QSPI_DATA_NONE;
			sCommand.DummyCycles = 0;

			if (HAL_QSPI_Command_IT(&hqspi, &sCommand) != HAL_OK)
			{
				Error_Handler();
			}

			step++;
			break;

		case 1:
			if(CmdCplt != 0)
			{
				CmdCplt = 0;
				StatusMatch = 0;

				/* Configure automatic polling mode to wait for end of erase ------- */
				QSPI_AutoPollingMemReady(&hqspi);
				//StatusMatch = 1;

				step++;
			}
			break;

		case 2:
			if(StatusMatch != 0)
			{
				StatusMatch = 0;
				TxCplt = 0;

				/* Wait write requst ----------------------------------------------- */
				uxBits = xEventGroupWaitBits(
						/* The event group being tested. */
						Flash_Event,
						/* The bits within the event group to wait for. */
						EVT_WRITEREQ | EVT_READREQ,
						/* BIT_0 & BIT_4 should be cleared before returning. */
						pdTRUE,
						/* Don't wait for both bits, either bit will do. */
						pdFALSE,
						/* Wait a maximum of 100ms for either bit to be set. */
						portMAX_DELAY );

				if(IS_EVT(EVT_WRITEREQ)){

					/* Enable write operations ----------------------------------------- */
					QSPI_WriteEnable(&hqspi);

					/* Writing Sequence ------------------------------------------------ */
					sCommand.Instruction = EXT_DUAL_IN_FAST_PROG_CMD;
					sCommand.AddressMode = QSPI_ADDRESS_2_LINES;
					sCommand.Address     = address;
					sCommand.DataMode    = QSPI_DATA_2_LINES;
					sCommand.NbData      = aTxBuffer_size;

					if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
					{
						Error_Handler();
					}

					if (HAL_QSPI_Transmit_DMA(&hqspi, (uint8_t *)aTxBuffer) != HAL_OK)
					{
						Error_Handler();
					}

					/* increment address */
					address 			 += aTxBuffer_size;
					step++;

				}


			}
			break;

		case 3:
			if(TxCplt != 0)
			{
				TxCplt = 0;
				StatusMatch = 0;

				/* Configure automatic polling mode to wait for end of program ----- */
				QSPI_AutoPollingMemReady(&hqspi);

				if (CHECK_END_ADDR(address)){
					step = 2;
				}else{
					/* rewrite all */
					step = 0;
				}
			}
			break;
		default :
			Error_Handler();
		}
	}
}


/**
 * @brief  This function send a Write Enable and wait it is effective.
 * @param  hqspi QSPI handle
 * @retval None
 */
static void QSPI_WriteEnable(QSPI_HandleTypeDef *hqspi)
{
	QSPI_CommandTypeDef     sCommand;
	QSPI_AutoPollingTypeDef sConfig;

	/* Enable write operations ------------------------------------------ */
	sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
	sCommand.Instruction       = WRITE_ENABLE_CMD;
	sCommand.AddressMode       = QSPI_ADDRESS_NONE;
	sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	sCommand.DataMode          = QSPI_DATA_NONE;
	sCommand.DummyCycles       = 0;
	sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
	sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

	if (HAL_QSPI_Command(hqspi, &sCommand, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		Error_Handler();
	}

	/* Configure automatic polling mode to wait for write enabling ---- */
	sConfig.Match           = 0x02;
	sConfig.Mask            = 0x02;
	sConfig.MatchMode       = QSPI_MATCH_MODE_AND;
	sConfig.StatusBytesSize = 1;
	sConfig.Interval        = 0x10;
	sConfig.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

	sCommand.Instruction    = READ_STATUS_REG_CMD;
	sCommand.DataMode       = QSPI_DATA_1_LINE;

	if (HAL_QSPI_AutoPolling(hqspi, &sCommand, &sConfig, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief  This function read the SR of the memory and wait the EOP.
 * @param  hqspi QSPI handle
 * @retval None
 */
static void QSPI_AutoPollingMemReady(QSPI_HandleTypeDef *hqspi)
{
	QSPI_CommandTypeDef     sCommand;
	QSPI_AutoPollingTypeDef sConfig;

	/* Configure automatic polling mode to wait for memory ready ------ */
	sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
	sCommand.Instruction       = READ_STATUS_REG_CMD;
	sCommand.AddressMode       = QSPI_ADDRESS_NONE;
	sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	sCommand.DataMode          = QSPI_DATA_1_LINE;
	sCommand.DummyCycles       = 0;
	sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
	sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

	sConfig.Match           = 0x00;
	sConfig.Mask            = 0x01;
	sConfig.MatchMode       = QSPI_MATCH_MODE_AND;
	sConfig.StatusBytesSize = 1;
	sConfig.Interval        = 0x10;
	sConfig.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

	if (HAL_QSPI_AutoPolling_IT(hqspi, &sCommand, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief  This function configure the dummy cycles on memory side.
 * @param  hqspi QSPI handle
 * @retval None
 */
static void QSPI_DummyCyclesCfg(QSPI_HandleTypeDef *hqspi)
{
	QSPI_CommandTypeDef sCommand;
	uint8_t reg;

	/* Read Volatile Configuration register --------------------------- */
	sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
	sCommand.Instruction       = READ_VOL_CFG_REG_CMD;
	sCommand.AddressMode       = QSPI_ADDRESS_NONE;
	sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	sCommand.DataMode          = QSPI_DATA_1_LINE;
	sCommand.DummyCycles       = 0;
	sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
	sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
	sCommand.NbData            = 1;

	if (HAL_QSPI_Command(hqspi, &sCommand, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_QSPI_Receive(hqspi, &reg, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		Error_Handler();
	}

	/* Enable write operations ---------------------------------------- */
	QSPI_WriteEnable(hqspi);

	/* Write Volatile Configuration register (with new dummy cycles) -- */
	sCommand.Instruction = WRITE_VOL_CFG_REG_CMD;
	MODIFY_REG(reg, 0xF0, (DUMMY_CLOCK_CYCLES_READ << POSITION_VAL(0xF0)));

	if (HAL_QSPI_Command(hqspi, &sCommand, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_QSPI_Transmit(hqspi, &reg, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		Error_Handler();
	}
}


void Flash_Write_Req (uint8_t *aTx, uint16_t size){

	if ((step == 2) && (size <= QSPI_PAGE_SIZE)){

		/* only if the flash is ready */
		aTxBuffer = aTx;
		aTxBuffer_size = size;

		EVENTSETBIT(Flash_Event,EVT_WRITEREQ);

	}else{

		aTxBuffer = 0;
		aTxBuffer_size = 0;

	}

}


uint16_t Int_Sensors_Stream (uint8_t *out, uint16_t size){

	/* Wait end of transfer */
	while(step != 2){
		osDelay(1);
	}

	if(address == 0){
		return 0;
	}

	if(Sen_dt_stream >= address){
		Sen_dt_stream = 0;
		/*  Erase Flash */
		step = 0;
		/* end of stream */
		return 0;
	}

	/* READ FROM FLASH */
	StatusMatch = 0;
	RxCplt = 0;

	/* Configure Volatile Configuration register (with new dummy cycles) */
	QSPI_DummyCyclesCfg(&hqspi);

	/* Reading Sequence ------------------------------------------------ */
	sCommand.Instruction = QUAD_OUT_FAST_READ_CMD;
	sCommand.AddressMode = QSPI_ADDRESS_1_LINE;
	sCommand.DataMode    = QSPI_DATA_4_LINES;
	sCommand.DummyCycles = DUMMY_CLOCK_CYCLES_READ;
	sCommand.Address     = Sen_dt_stream;
	sCommand.NbData      = size;

	if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_QSPI_Receive_DMA(&hqspi, out) != HAL_OK)
	{
		Error_Handler();
	}

	/* whait end of transfer */
	while(RxCplt == 0);
	RxCplt = 0;



	Sen_dt_stream += (uint32_t)size;

	return size;

}


#pragma GCC push_options
#pragma GCC optimize ("O3")

/**
 * @brief  Command completed callbacks.
 * @param  hqspi QSPI handle
 * @retval None
 */
void HAL_QSPI_CmdCpltCallback(QSPI_HandleTypeDef *hqspi)
{
	CmdCplt++;
}

/**
 * @brief  Rx Transfer completed callbacks.
 * @param  hqspi QSPI handle
 * @retval None
 */
void HAL_QSPI_RxCpltCallback(QSPI_HandleTypeDef *hqspi)
{
	RxCplt++;
}

/**
 * @brief  Tx Transfer completed callbacks.
 * @param  hqspi QSPI handle
 * @retval None
 */
void HAL_QSPI_TxCpltCallback(QSPI_HandleTypeDef *hqspi)
{
	TxCplt++;
}

/**
 * @brief  Status Match callbacks
 * @param  hqspi QSPI handle
 * @retval None
 */
void HAL_QSPI_StatusMatchCallback(QSPI_HandleTypeDef *hqspi)
{
	StatusMatch++;
}

#pragma GCC pop_options
