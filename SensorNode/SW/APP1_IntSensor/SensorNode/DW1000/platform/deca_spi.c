/*! ----------------------------------------------------------------------------
 * @file	deca_spi.c
 * @brief	SPI access functions
 *
 * @attention
 *
 * Copyright 2015 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */

#include "main.h"
#include "deca_spi.h"
#include "deca_device_api.h"
#include "port.h"
#include "spi.h"

extern DMA_HandleTypeDef hdma_spi2_rx;
extern DMA_HandleTypeDef hdma_spi2_tx;

extern  void memcpy_DMA (uint32_t *SrcAddress, uint32_t *DstAddress, uint32_t DataLength);
extern  void memcpy_DMA_poll (void);


/****************************************************************************//**
 *
 * 								DW1000 SPI section
 *
 *******************************************************************************/
/*! ------------------------------------------------------------------------------------------------------------------
 * Function: openspi()
 *
 * Low level abstract function to open and initialise access to the SPI device.
 * returns 0 for success, or -1 for error
 */
int openspi(/*SPI_TypeDef* SPIx*/)
{
	return 0;
} // end openspi()

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: closespi()
 *
 * Low level abstract function to close the the SPI device.
 * returns 0 for success, or -1 for error
 */
int closespi(void)
{
	return 0;
} // end closespi()

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: writetospi()
 *
 * Low level abstract function to write to the SPI
 * Takes two separate byte buffers for write header and write data
 * returns 0 for success, or -1 for error
 */
#pragma GCC optimize ("O3")
int writetospi(uint16_t headerLength,
			   const	uint8_t *headerBuffer,
			   uint32_t bodyLength,
			   const	uint8_t *bodyBuffer)
{
//    decaIrqStatus_t  stat ;
//    stat = decamutexon() ;

	while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);

    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET); /**< Put chip select line low */

    HAL_SPI_Transmit(&hspi2, (uint8_t *)&headerBuffer[0], headerLength, 10);	/* Send header in polling mode */
    if (bodyLength < 10){
    	HAL_SPI_Transmit(&hspi2, (uint8_t *)&bodyBuffer[0], bodyLength, 10);		/* Send data in polling mode */
    }else{
    	HAL_SPI_Transmit_DMA(&hspi2,(uint8_t *)&bodyBuffer[0], bodyLength);		/* Send data in DMA mode */
    	while ((HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY) &&
    			(HAL_DMA_PollForTransfer(&hdma_spi2_tx, HAL_DMA_FULL_TRANSFER, 0) != HAL_OK));
    }

    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET); /**< Put chip select line high */


//    decamutexoff(stat) ;

    return 0;
} // end writetospi()


/*! ------------------------------------------------------------------------------------------------------------------
 * Function: readfromspi()
 *
 * Low level abstract function to read from the SPI
 * Takes two separate byte buffers for write header and read data
 * returns the offset into read buffer where first byte of read data may be found,
 * or returns -1 if there was an error
 */
#pragma GCC optimize ("O3")
int readfromspi(uint16_t headerLength,
				const uint8_t *headerBuffer,
				uint32_t readlength,
				uint8_t *readBuffer)
{
	uint8_t spi_TmpBuffer[BUFFLEN];
	assert_param(headerLength+readlength < BUFFLEN );
	
//    decaIrqStatus_t  stat ;
//    stat = decamutexon() ;

	/* Blocking: Check whether previous transfer has been finished */
	while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);

	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET); /**< Put chip select line low */

	if (readlength < 10){
		HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)headerBuffer, spi_TmpBuffer, (uint16_t)(headerLength+readlength), 10);
	}else{
		HAL_SPI_TransmitReceive_DMA(&hspi2,(uint8_t *)headerBuffer, spi_TmpBuffer, (uint16_t)(headerLength+readlength));
		while ((HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY) &&
		    			(HAL_DMA_PollForTransfer(&hdma_spi2_tx, HAL_DMA_FULL_TRANSFER, 0) != HAL_OK) &&
						(HAL_DMA_PollForTransfer(&hdma_spi2_rx, HAL_DMA_FULL_TRANSFER, 0) != HAL_OK));
	}
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET); /**< Put chip select line high */

	memcpy((uint8_t*)readBuffer , (uint8_t*)&spi_TmpBuffer[headerLength], readlength);
	//memcpy_DMA ((uint32_t*)&spi_TmpBuffer[headerLength],(uint32_t*)readBuffer,readlength);
	//memcpy_DMA_poll();

//	decamutexoff(stat);

    return 0;
} // end readfromspi()

/****************************************************************************//**
 *
 * 								END OF DW1000 SPI section
 *
 *******************************************************************************/

