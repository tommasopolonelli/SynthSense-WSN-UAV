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

#include "deca_spi.h"
#include "deca_device_api.h"
#include "port.h"

extern 	SPI_HandleTypeDef hspi1;	/*clocked from 72MHz*/
extern	SPI_HandleTypeDef hspi2;	/*clocked from 36MHz*/
extern  DMA_HandleTypeDef hdma_spi1_tx;
extern  DMA_HandleTypeDef hdma_spi1_rx;
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

	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);

    HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_RESET); /**< Put chip select line low */

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&headerBuffer[0], headerLength, 10);	/* Send header in polling mode */
    if (bodyLength < 10){
    	HAL_SPI_Transmit(&hspi1, (uint8_t *)&bodyBuffer[0], bodyLength, 10);		/* Send data in polling mode */
    }else{
    	HAL_SPI_Transmit_DMA(&hspi1,(uint8_t *)&bodyBuffer[0], bodyLength);		/* Send data in DMA mode */
    	while ((HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY) &&
    			(HAL_DMA_PollForTransfer(&hdma_spi1_tx, HAL_DMA_FULL_TRANSFER, 0) != HAL_OK));
    }

    HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_SET); /**< Put chip select line high */


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
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);

	HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_RESET); /**< Put chip select line low */

	if (readlength < 10){
		HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)headerBuffer, spi_TmpBuffer, (uint16_t)(headerLength+readlength), 10);
	}else{
		HAL_SPI_TransmitReceive_DMA(&hspi1,(uint8_t *)headerBuffer, spi_TmpBuffer, (uint16_t)(headerLength+readlength));
		while ((HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY) &&
		    			(HAL_DMA_PollForTransfer(&hdma_spi1_tx, HAL_DMA_FULL_TRANSFER, 0) != HAL_OK) &&
						(HAL_DMA_PollForTransfer(&hdma_spi1_rx, HAL_DMA_FULL_TRANSFER, 0) != HAL_OK));
	}
	HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_SET); /**< Put chip select line high */

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


#if (EVB1000_LCD_SUPPORT == 1)
#pragma GCC optimize ("O3")
void writetoLCD
(
    uint32       bodylength,
    uint8        rs_enable,
    const uint8 *bodyBuffer
)
{

	while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);

	int sleep = 0;

	if(rs_enable)
	{
    	port_LCD_RS_set();
    }
	else
	{
		if(bodylength == 1)
		{
			if(bodyBuffer[0] & 0x3) //if this is command = 1 or 2 - execution time is > 1ms
				sleep = 1 ;
		}
    	port_LCD_RS_clear();
    }

	port_SPIy_clear_chip_select();  //CS low for SW controllable SPI_NSS

    HAL_SPI_Transmit(&hspi2, (uint8_t*)bodyBuffer , bodylength, 10);
	
	

    port_LCD_RS_clear();
    port_SPIy_set_chip_select();  //CS high for SW controllable SPI_NSS

    if(sleep)
    	Sleep(2);
} // end writetoLCD()
#endif

