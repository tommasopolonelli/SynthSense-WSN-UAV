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

/* N25Q064A Micron memory */
/* Size of the flash */
#define QSPI_FLASH_SIZE                      22
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
#define QSPI_END_ADDR              			(1 << QSPI_FLASH_SIZE)

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Buffer used for transmission */
static const uint8_t aTxBuffer[] = " ****QSPI communication based on DMA****  ****QSPI communication based on DMA****  ****QSPI communication based on DMA****  ****QSPI communication based on DMA****  ****QSPI communication based on DMA****  ****QSPI communication based on DMA**** ";
static uint8_t aRxBuffer[sizeof(aTxBuffer)+10];
static uint8_t CmdCplt, RxCplt, TxCplt, StatusMatch;

/* Private function prototypes -----------------------------------------------*/

static void QSPI_DummyCyclesCfg(QSPI_HandleTypeDef *hqspi);
static void QSPI_AutoPollingMemReady(QSPI_HandleTypeDef *hqspi);
static void QSPI_WriteEnable(QSPI_HandleTypeDef *hqspi);
/* Private user code ---------------------------------------------------------*/

void FlashMngm_Task (void const * argument){

	uint8_t start = (uint8_t)((uint8_t *)argument);
	QSPI_CommandTypeDef sCommand;
	uint32_t address = 0;
	uint16_t index;
	uint8_t step = 0;

	/* task disabled */
	if (!start){
		/*Configure GPIO pin Output Level */
		HAL_GPIO_WritePin(PW_FLASH_GPIO_Port, PW_FLASH_Pin, GPIO_PIN_RESET);

		GPIO_InitTypeDef GPIO_InitStruct = {0};
	    GPIO_InitStruct.Pin = GPIO_PIN_8;
	    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	    GPIO_InitStruct.Pin = GPIO_PIN_9;
	    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7;
	    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		/* delete task */
		vTaskDelete( osThreadGetId () );
		return;
	}

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(PW_FLASH_GPIO_Port, PW_FLASH_Pin, GPIO_PIN_SET);
	osDelay(10);

	MX_QUADSPI_Init();

	/* start application */
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,0);
	while (1){
		 /* USER CODE BEGIN WHILE */
		  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
		  sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
		  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
		  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
		  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

		  while (1)
		  {
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,0);
		    switch(step)
		    {
		      case 0:
		        CmdCplt = 0;

		        /* Initialize Reception buffer --------------------------------------- */
		        /*for (index = 0; index < sizeof(aRxBuffer); index++)
		        {
		          aRxBuffer[index] = 0;
		        }*/

		        /* Enable write operations ------------------------------------------- */
		        QSPI_WriteEnable(&hqspi);

		        /* Erasing Sequence -------------------------------------------------- */
		        sCommand.Instruction = SUBSECTOR_ERASE_CMD;
		        sCommand.AddressMode = QSPI_ADDRESS_1_LINE;
		        sCommand.Address     = address;
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

		          /* Enable write operations ----------------------------------------- */
		          QSPI_WriteEnable(&hqspi);

		          /* Writing Sequence ------------------------------------------------ */
		          sCommand.Instruction = EXT_DUAL_IN_FAST_PROG_CMD;
		          sCommand.AddressMode = QSPI_ADDRESS_2_LINES;
		          sCommand.DataMode    = QSPI_DATA_2_LINES;
		          sCommand.NbData      = sizeof(aTxBuffer);

		          if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
		          {
		        	  Error_Handler();
		          }

		          if (HAL_QSPI_Transmit_DMA(&hqspi, (uint8_t *)aTxBuffer) != HAL_OK)
		          {
		        	  Error_Handler();
		          }

		          step++;
		        }
		        break;

		      case 3:
		        if(TxCplt != 0)
		        {
		          TxCplt = 0;
		          StatusMatch = 0;

		          /* Configure automatic polling mode to wait for end of program ----- */
		          QSPI_AutoPollingMemReady(&hqspi);
		          //StatusMatch = 1;

		          step++;
		        }
		        break;

		      case 4:
		        if(StatusMatch != 0)
		        {
		          StatusMatch = 0;
		          RxCplt = 0;

		          /* Configure Volatile Configuration register (with new dummy cycles) */
		          QSPI_DummyCyclesCfg(&hqspi);

		          /* Reading Sequence ------------------------------------------------ */
		          sCommand.Instruction = QUAD_OUT_FAST_READ_CMD;
		          sCommand.AddressMode = QSPI_ADDRESS_1_LINE;
		          sCommand.DataMode    = QSPI_DATA_4_LINES;
		          sCommand.DummyCycles = DUMMY_CLOCK_CYCLES_READ;

		          if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
		          {
		            Error_Handler();
		          }

		          if (HAL_QSPI_Receive_DMA(&hqspi, aRxBuffer) != HAL_OK)
		          {
		            Error_Handler();
		          }
		          step++;
		        }
		        break;

		      case 5:
		        if (RxCplt != 0)
		        {
		          RxCplt = 0;

		          /* Result comparison ----------------------------------------------- */
		          /*for (index = 0; index < sizeof(aTxBuffer); index++)
		          {
		            if (aRxBuffer[index] != aTxBuffer[index])
		            {
		              //BSP_LED_On(LED_RED);
		            }
		          }*/
		          //BSP_LED_Toggle(LED_GREEN);

		          //KILL THE TASK
		          /*Configure GPIO pin Output Level */
		          //HAL_GPIO_WritePin(PW_FLASH_GPIO_Port, PW_FLASH_Pin, GPIO_PIN_RESET);
		          //HAL_QSPI_MspDeInit(&hqspi);
		          /* delete task */
		          //vTaskDelete( osThreadGetId () );
		          //return;

		          address += QSPI_PAGE_SIZE;
		          if(address >= QSPI_END_ADDR)
		          {
		            address = 0;
		          }

		          step = 0;

		          osDelay(500);
		          StatusMatch = 1;
		          step = 2;
		        }
		        break;

		      default :
		        Error_Handler();
		    }
		    //osDelay(1);
		    //HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
		  }
		osDelay(1000);
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
