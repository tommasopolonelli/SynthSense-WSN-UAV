/*! ----------------------------------------------------------------------------
 * @file	port.c
 * @brief	HW specific definitions and functions for portability
 *
 * @attention
 *
 * Copyright 2016 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */
#include "macro.h"
#include "port.h"
#include "deca_device_api.h"
#include "gpio.h"
#include "spi.h"
#include "cmsis_os.h"
#include "instance.h"


/****************************************************************************//**
 *
 * 								APP global variables
 *
 *******************************************************************************/


/****************************************************************************//**
 *
 * 					Port private variables and function prototypes
 *
 *******************************************************************************/
static volatile uint32_t signalResetDone;
static void port_RTOS_CreateEvent (void);

/****************************************************************************//**
 *
 * 								Time section
 *
 *******************************************************************************/
 
/* @fn 	  portGetTickCnt
 * @brief wrapper for to read a SysTickTimer, which is incremented with
 * 		  CLOCKS_PER_SEC frequency.
 * 		  The resolution of time32_incr is usually 1/1000 sec.
 * */
__INLINE uint32_t
portGetTickCnt(void)
{
	return HAL_GetTick();
}


/* @fn	  usleep
 * @brief precise usleep() delay
 * */
#pragma GCC optimize ("O0")
int usleep(useconds_t usec)
{
	int i,j;
#pragma GCC ivdep
	for(i=0;i<usec;i++)
	{
#pragma GCC ivdep
		for(j=0;j<2;j++)
		{
			__NOP();
			__NOP();
		}
	}
	return 0;
}


/* @fn 	  Sleep
 * @brief Sleep delay in ms using SysTick timer
 * */
__INLINE void
Sleep(uint32_t x)
{
	osDelay(x);
	//HAL_Delay(x);
}

/****************************************************************************//**
 *
 * 								END OF Time section
 *
 *******************************************************************************/

/****************************************************************************//**
 *
 * 								Configuration section
 *
 *******************************************************************************/

/* @fn 	  peripherals_init
 * */
int peripherals_init (void)
{
	port_RTOS_CreateEvent ();
	/* All has been initialized in the CubeMx code, see main.c */
	return 0;
}


/* @fn 	  spi_peripheral_init
 * */
void spi_peripheral_init()
{

	/* SPI's has been initialized in the CubeMx code, see main.c */

	port_LCD_RS_clear();

	port_LCD_RW_clear();
}



/**
  * @brief  Checks whether the specified EXTI line is enabled or not.
  * @param  EXTI_Line: specifies the EXTI line to check.
  *   This parameter can be:
  *     @arg EXTI_Linex: External interrupt line x where x(0..19)
  * @retval The "enable" state of EXTI_Line (SET or RESET).
  */
ITStatus EXTI_GetITEnStatus(uint32_t x)
{
	return ((NVIC->ISER[(((uint32_t)x) >> 5UL)] &\
		    (uint32_t)(1UL << (((uint32_t)x) & 0x1FUL)) ) == (uint32_t)RESET)?(RESET):(SET);
}
/****************************************************************************//**
 *
 * 							End of configuration section
 *
 *******************************************************************************/

/****************************************************************************//**
 *
 * 							DW1000 port section
 *
 *******************************************************************************/

/* @fn		reset_DW1000
 * @brief	DW_RESET pin on DW1000 has 2 functions
 * 			In general it is output, but it also can be used to reset the digital
 * 			part of DW1000 by driving this pin low.
 * 			Note, the DW_RESET pin should not be driven high externally.
 * */
void reset_DW1000(void)
{

	// default config
	// Enable GPIO used for DW1000 reset as open collector output
	/*GPIO_InitStruct.Pin = DW_RESET_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(DW_RESET_GPIO_Port, &GPIO_InitStruct);*/

	//drive the RSTn pin low
	HAL_GPIO_WritePin(DW_RST_GPIO_Port, DW_RST_Pin, GPIO_PIN_RESET);

	usleep(10);

	//put the pin back to output open-drain (not active)
	setup_DW1000RSTnIRQ(0);

	Sleep(2);

}

/* @fn		setup_DW1000RSTnIRQ
 * @brief	setup the DW_RESET pin mode
 * 			0 - output Open collector mode
 * 			!0 - input mode with connected EXTI0 IRQ
 * */
void setup_DW1000RSTnIRQ(int enable)
{
	//GPIO_InitTypeDef GPIO_InitStruct;

	if(enable)
	{
		// Enable GPIO used as DECA RESET for interrupt
		//GPIO_InitStruct.Pin = DW_RESET_Pin;
		//GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
		//GPIO_InitStruct.Pull = GPIO_NOPULL;
		//HAL_GPIO_Init(DW_RESET_GPIO_Port, &GPIO_InitStruct);

		HAL_NVIC_EnableIRQ(EXTI0_IRQn);		//pin #0 -> EXTI #0
		//HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
	}
	else
	{
		HAL_NVIC_DisableIRQ(EXTI0_IRQn);	//pin #0 -> EXTI #0

		//put the pin back to tri-state ... as 
		//output open-drain (not active)
		//GPIO_InitStruct.Pin = DW_RESET_Pin;
		//GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
		//GPIO_InitStruct.Pull = GPIO_NOPULL;
		//GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		//HAL_GPIO_Init(DW_RESET_GPIO_Port, &GPIO_InitStruct);
		HAL_GPIO_WritePin(DW_RST_GPIO_Port, DW_RST_Pin, GPIO_PIN_SET);
	}
}


/* @fn		port_is_boot1_low
 * @brief	check the BOOT1 pin status.
 * @return  1 if ON and 0 for OFF
 * */
int port_is_boot1_low(void)
{
	/* useless in standalone applications */
	//return ((GPIO_ReadInputDataBit(TA_BOOT1_GPIO, TA_BOOT1))?(0):(1));
	return 1;
}

/* @fn		port_is_boot1_low
 * @brief	check the BOOT1 pin status.
 * @return  1 if ON and 0 for OFF
 * */
int port_is_boot1_on(uint16_t x)
{
	/* useless in standalone applications */
	//return ((GPIO_ReadInputDataBit(TA_BOOT1_GPIO, TA_BOOT1))?(0):(1));
	return 1;
}

/* @fn		port_is_switch_on
 * @brief	check the switch status.
 * 			when switch (S1) is 'on' the pin is low
 * @return  1 if ON and 0 for OFF
 * */
int port_is_switch_on(uint16_t GPIOpin)
{
	//return ((GPIO_ReadInputDataBit(TA_SW1_GPIO, GPIOpin))?(0):(1));

	/* static configurations */
	switch(GPIOpin){
	case TA_SW1_3:
		return 1;
		break;
	case TA_SW1_4:
		return SWS_ANC_MODE;
		break;
	case TA_SW1_5:
		return SWS_SHF_MODE;
		break;
	case TA_SW1_6:
		return SWS_64M_MODE;
		break;
	case TA_SW1_7:
		return SWS_CH5_MODE;
		break;
	case TA_SW1_8:
		return SWS_TXSPECT_MODE;
		break;
	default:
		return 0;
		break;
	}
}


/* @fn		led_off
 * @brief	switch off the led from led_t enumeration
 * */
void led_off (led_t led)
{
	switch (led)
	{
	case LED_PC6:
	case LED_PC7:
	case LED_PC8:
	case LED_PC9:
	case LED_ALL:
		GPIO_ResetBits(LED_GPIO_Port, LED_Pin);
		break;
	default:
		// do nothing for undefined led number
		break;
	}
}

/* @fn		led_on
 * @brief	switch on the led from led_t enumeration
 * */
void led_on (led_t led)
{
	switch (led)
	{
	case LED_PC6:
	case LED_PC7:
	case LED_PC8:
	case LED_PC9:
	case LED_ALL:
		GPIO_SetBits(LED_GPIO_Port, LED_Pin);
		break;
	default:
		// do nothing for undefined led number
		break;
	}
}


/* @fn		port_wakeup_dw1000
 * @brief	"slow" waking up of DW1000 using DW_CS only
 * */
void port_wakeup_dw1000(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	/* standard procedure */
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
    Sleep(1);
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
    Sleep(7);						//wait 7ms for DW1000 XTAL to stabilise

#ifdef LOW_POW_LISTENING
    /* WAKE UP FROM LPL */
    GPIO_InitStruct.Pin = DW_RST_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DW_RST_GPIO_Port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(DW_WU_GPIO_Port, DW_WU_Pin, GPIO_PIN_SET);

    /* wait until the DW1000 is ready */
    while(HAL_GPIO_ReadPin(DW_RST_GPIO_Port, DW_RST_Pin) == 0);
    /* set the RESET pin as default mode */
    HAL_GPIO_WritePin(DW_RST_GPIO_Port, DW_RST_Pin, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = DW_RST_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DW_RST_GPIO_Port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(DW_RST_GPIO_Port, DW_RST_Pin, GPIO_PIN_SET);
    /* Immediately send a reset over SPI */
    dwt_softreset();
    dwt_rxenable(0);
    dwt_configuresleep( 0, DWT_WAKE_CS | DWT_WAKE_WK);
    dwt_rxreset();
    usleep(10);
    /* Force wake up is not longer needed */
    HAL_GPIO_WritePin(DW_WU_GPIO_Port, DW_WU_Pin, GPIO_PIN_RESET);

    /* NO WAIT since the DW1000 is already waked up */
    //Sleep(7);

#endif

}

/* @fn		port_wakeup_dw1000_fast
 * @brief	waking up of DW1000 using DW_CS and DW_RESET pins.
 * 			The DW_RESET signalling that the DW1000 is in the INIT state.
 * 			the total fast wakeup takes ~2.2ms and depends on crystal startup time
 * */
void port_wakeup_dw1000_fast(void)
{
	#define WAKEUP_TMR_MS	(10)

	uint32_t x = 0;
	uint32_t timestamp = HAL_GetTick();	//protection

	setup_DW1000RSTnIRQ(0); 		//disable RSTn IRQ
	signalResetDone = 0;			//signalResetDone connected to RST_PIN_IRQ
	setup_DW1000RSTnIRQ(1); 		//enable RSTn IRQ
	port_SPIx_clear_chip_select();  //CS low

	//need to poll to check when the DW1000 is in the IDLE, the CPLL interrupt is not reliable
	//when RSTn goes high the DW1000 is in INIT, it will enter IDLE after PLL lock (in 5 us)
	while((signalResetDone == 0) && \
		  ((HAL_GetTick() - timestamp) < WAKEUP_TMR_MS))
	{
		x++;	 //when DW1000 will switch to an IDLE state RSTn pin will high
	}
	setup_DW1000RSTnIRQ(0); 		//disable RSTn IRQ
	port_SPIx_set_chip_select();  	//CS high

	//it takes ~35us in total for the DW1000 to lock the PLL, download AON and go to IDLE state
	usleep(35);
}



/* @fn		port_set_dw1000_slowrate
 * @brief	set 2.25MHz
 * 			note: hspi2 is clocked from 72MHz
 * */
void port_set_dw1000_slowrate(void)
{
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
	HAL_SPI_Init(&hspi2);
}

/* @fn		port_set_dw1000_fastrate
 * @brief	set 18MHz
 * 			note: hspi2 is clocked from 72MHz
 * */
void port_set_dw1000_fastrate(void)
{
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
	HAL_SPI_Init(&hspi2);
}

/* @fn		port_LCD_RS_set
 * @brief	wrapper to set LCD_RS pin
 * */
void port_LCD_RS_set(void)
{
	//HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);
}

/* @fn		port_LCD_RS_clear
 * @brief	wrapper to clear LCD_RS pin
 * */
void port_LCD_RS_clear(void)
{
	//HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET);
}

/* @fn		port_LCD_RW_clear
 * @brief	wrapper to set LCD_RW pin
 * */
void port_LCD_RW_set(void)
{
	//HAL_GPIO_WritePin(LCD_RW_GPIO_Port, LCD_RW_Pin, GPIO_PIN_SET);
}

/* @fn		port_LCD_RW_clear
 * @brief	wrapper to clear LCD_RW pin
 * */
void port_LCD_RW_clear(void)
{
	//HAL_GPIO_WritePin(LCD_RW_GPIO_Port, LCD_RW_Pin, GPIO_PIN_RESET);
}

/****************************************************************************//**
 *
 * 							End APP port section
 *
 *******************************************************************************/



/****************************************************************************//**
 *
 * 								IRQ section
 *
 *******************************************************************************/

/* @fn		HAL_GPIO_EXTI_Callback
 * @brief	IRQ HAL call-back for all EXTI configured lines
 * 			i.e. DW_RESET_Pin and DW_IRQn_Pin
 * */
void DW1000_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == DW_RST_Pin)
	{
		signalResetDone = 1;
	}
	else if (GPIO_Pin == DW_IRQ_Pin)
	{
		process_deca_irq();
	}
	else
	{
	}
}

/* @fn		process_deca_irq
 * @brief	main call-back for processing of DW1000 IRQ
 * 			it re-enters the IRQ routing and processes all events.
 * 			After processing of all events, DW1000 will clear the IRQ line.
 * */
__INLINE void process_deca_irq(void)
{
	while(port_CheckEXT_IRQ() != 0)
	{

    	dwt_isr();

    } //while DW1000 IRQ line active
}


/* @fn		port_DisableEXT_IRQ
 * @brief	wrapper to disable DW_IRQ pin IRQ
 * 			in current implementation it disables all IRQ from lines 5:9
 * */
__INLINE void port_DisableEXT_IRQ(void)
{
	NVIC_DisableIRQ(DW_IRQ_EXTI_IRQn);
	NVIC_ClearPendingIRQ(DW_IRQ_EXTI_IRQn);
}

/* @fn		port_EnableEXT_IRQ
 * @brief	wrapper to enable DW_IRQ pin IRQ
 * 			in current implementation it enables all IRQ from lines 5:9
 * */
__INLINE void port_EnableEXT_IRQ(void)
{
	NVIC_EnableIRQ(DW_IRQ_EXTI_IRQn);
}


/* @fn		port_GetEXT_IRQStatus
 * @brief	wrapper to read a DW_IRQ pin IRQ status
 * */
__INLINE uint32_t port_GetEXT_IRQStatus(void)
{
	return EXTI_GetITEnStatus(DW_IRQ_EXTI_IRQn);
}


/* @fn		port_CheckEXT_IRQ
 * @brief	wrapper to read DW_IRQ input pin state
 * */
__INLINE uint32_t port_CheckEXT_IRQ(void)
{
	return HAL_GPIO_ReadPin(DW_IRQ_GPIO_Port, DW_IRQ_Pin);
}


/****************************************************************************//**
 *
 * 								END OF IRQ section
 *
 *******************************************************************************/

/****************************************************************************//**
 *
 * 								RTOS report section
 *
 *******************************************************************************/
enum {
  EVT_DW1000  						= (1 << 0), /* 0b0000000000000001 */
  EVT_DW1001               			= (1 << 1) /* 0b0000000000000010 */
};

static StaticEventGroup_t dw1000_EventGroupBuffer;
EventGroupHandle_t dw1000_Event;

static void port_RTOS_CreateEvent (void){
// Create the event group
	dw1000_Event = xEventGroupCreateStatic( &dw1000_EventGroupBuffer );
}

void port_RTOS_WaitEvent( void ){
	xEventGroupWaitBits(
				/* The event group being tested. */
				dw1000_Event,
				/* The bits within the event group to wait for. */
				EVT_DW1000,
				/* BIT_0 & BIT_4 should be cleared before returning. */
				pdTRUE,
				/* Don't wait for both bits, either bit will do. */
				pdFALSE,
				/* Wait a maximum of 100ms for either bit to be set. */
				portMAX_DELAY );
}

#pragma GCC push_options
#pragma GCC optimize ("O3")

void port_RTOS_SetEvent( void ){

	EVENTSETBIT(dw1000_Event,EVT_DW1000);

}

#pragma GCC pop_options
/****************************************************************************//**
 *
 * 						   END RTOS report section
 *
 *******************************************************************************/

#if __ENABLE_USB__

/****************************************************************************//**
 *
 * 								USB report section
 *
 *******************************************************************************/
#include "usb_device.h"

#define REPORT_BUFSIZE	0x2000

extern USBD_HandleTypeDef  hUsbDeviceFS;

static struct
{
	HAL_LockTypeDef       Lock;		/*!< locking object                  */
}
txhandle={.Lock = HAL_UNLOCKED};

static char 	rbuf[REPORT_BUFSIZE];				/**< circular report buffer, data to be transmitted in flush_report_buff() Thread */
static struct 	circ_buf report_buf = {	.buf = rbuf,
										.head= 0,
										.tail= 0};

static uint8_t 	ubuf[CDC_DATA_FS_MAX_PACKET_SIZE];	/**< used to transmit new chunk of data in single USB flush */

/* @fn 		port_tx_msg()
 * @brief 	put message to circular report buffer
 * 			it will be transmitted in background ASAP from flushing Thread
 * @return	HAL_BUSY - can not do it now, wait for release
 * 			HAL_ERROR- buffer overflow
 * 			HAL_OK   - scheduled for transmission
 * */
HAL_StatusTypeDef port_tx_msg(uint8_t	*str, int  len)
{
	int head, tail, size;
	HAL_StatusTypeDef	ret;

	/* add TX msg to circular buffer and increase circular buffer length */

	__HAL_LOCK(&txhandle);	//return HAL_BUSY if locked
	head = report_buf.head;
	tail = report_buf.tail;
	__HAL_UNLOCK(&txhandle);

	size = REPORT_BUFSIZE;

	if(CIRC_SPACE(head, tail, size) > (len))
	{
		while (len > 0)
		{
			report_buf.buf[head]= *(str++);
			head= (head+1) & (size - 1);
			len--;
		}

		__HAL_LOCK(&txhandle);	//return HAL_BUSY if locked
		report_buf.head = head;
		__HAL_UNLOCK(&txhandle);

#ifdef CMSIS_RTOS
		osSignalSet(usbTxTaskHandle, signalUsbFlush);	//RTOS multitasking signal start flushing
#endif
		ret = HAL_OK;
	}
	else
	{
		/* if packet can not fit, setup TX Buffer overflow ERROR and exit */
		ret = HAL_ERROR;
	}

    return ret;
}


/* @fn		flush_report_buff
 * @brief 	FLUSH should have higher priority than port_tx_msg()
 * 			it shall be called periodically from process, which can not be locked,
 * 			i.e. from independent high priority thread
 * */
HAL_StatusTypeDef flush_report_buff(void)
{
	USBD_CDC_HandleTypeDef   *hcdc = (USBD_CDC_HandleTypeDef*)(hUsbDeviceFS.pClassData);

	int i, head, tail, len, size = REPORT_BUFSIZE;

	__HAL_LOCK(&txhandle);	//"return HAL_BUSY;" if locked
	head = report_buf.head;
	tail = report_buf.tail;
	__HAL_UNLOCK(&txhandle);

	len = CIRC_CNT(head, tail, size);

	if( len > 0 )
	{
		/*	check USB status - ready to TX */
		if((hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) || (hcdc->TxState != 0))
		{
			return HAL_BUSY;	/**< USB is busy. Let it send now, will return next time */
		}


		/* copy MAX allowed length from circular buffer to non-circular TX buffer */
		len = MIN(CDC_DATA_FS_MAX_PACKET_SIZE, len);

		for(i=0; i<len; i++)
		{
			ubuf[i] = report_buf.buf[tail];
			tail = (tail + 1) & (size - 1);
		}

		__HAL_LOCK(&txhandle);	//"return HAL_BUSY;" if locked
		report_buf.tail = tail;
		__HAL_UNLOCK(&txhandle);

		/* setup USB IT transfer */
		if(CDC_Transmit_FS(ubuf, (uint16_t)len) != USBD_OK)
		{
			/**< indicate USB transmit error */
		}
	}

	return HAL_OK;
}

/****************************************************************************//**
 *
 * 								END OF Report section
 *
 *******************************************************************************/
 
#endif
