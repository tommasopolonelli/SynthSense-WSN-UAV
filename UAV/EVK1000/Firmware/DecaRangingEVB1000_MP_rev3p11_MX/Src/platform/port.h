/*! ----------------------------------------------------------------------------
 * @file	port.h
 * @brief	HW specific definitions and functions for portability
 *
 * @attention
 *
 * Copyright 2015 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */


#ifndef PORT_H_
#define PORT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>
#include "compiler.h"

#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"

#define BUFFLEN 	(4096+128)

#define BUF_SIZE	(64)

#define USB_SUPPORT

typedef struct
{
    uint16_t		usblen;					/**< for RX from USB */
    uint8_t			usbbuf[BUF_SIZE*20];		/**< for RX from USB */
}__packed app_t;


extern app_t	app;


/*****************************************************************************************************************//*
**/

 /****************************************************************************//**
  *
  * 								Types definitions
  *
  *******************************************************************************/
typedef uint64_t        uint64 ;

typedef int64_t         int64 ;


#ifndef FALSE
#define FALSE               0
#endif

#ifndef TRUE
#define TRUE                1
#endif

typedef enum
{
    LED_PC6, //LED5
    LED_PC7, //LED6
    LED_PC8, //LED7
    LED_PC9, //LED8
    LED_ALL,
    LEDn
} led_t;

/****************************************************************************//**
 *
 * 								MACRO
 *
 *******************************************************************************/


#if !(EXTI9_5_IRQn)
#define DECAIRQ_EXTI_IRQn		(23)
#else
#define DECAIRQ_EXTI_IRQn		(EXTI9_5_IRQn)
#endif

#if !(EXTI0_IRQn)
#define EXTI0_IRQn		(6)
#endif



#define DW1000_RSTn					DW_RESET_Pin
#define DW1000_RSTn_GPIO			DW_RESET_GPIO_Port


#define DECAIRQ                     DW_IRQn_Pin
#define DECAIRQ_GPIO                DW_IRQn_GPIO_Port

#define TA_BOOT1                 	GPIO_PIN_2
#define TA_BOOT1_GPIO            	GPIOB

#define TA_RESP_DLY                 GPIO_PIN_0
#define TA_RESP_DLY_GPIO            GPIOC

#define TA_SW1_3					GPIO_PIN_0
#define TA_SW1_4					GPIO_PIN_1
#define TA_SW1_5					GPIO_PIN_2
#define TA_SW1_6					GPIO_PIN_3
#define TA_SW1_7					GPIO_PIN_4
#define TA_SW1_8					GPIO_PIN_5
#define TA_SW1_GPIO                 GPIOC

/****************************************************************************//**
 *
 * 								MACRO function
 *
 *******************************************************************************/

#define GPIO_ResetBits(x,y)				HAL_GPIO_WritePin(x,y, RESET)
#define GPIO_SetBits(x,y)				HAL_GPIO_WritePin(x,y, SET)
#define GPIO_ReadInputDataBit(x,y) 		HAL_GPIO_ReadPin (x,y)


/* NSS pin is SW controllable */
#define port_SPIx_set_chip_select()		HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_SET)
#define port_SPIx_clear_chip_select()	HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_RESET)

/* NSS pin is SW controllable */
#define port_SPIy_set_chip_select()		HAL_GPIO_WritePin(LCD_NSS_GPIO_Port, LCD_NSS_Pin, GPIO_PIN_SET)
#define port_SPIy_clear_chip_select()	HAL_GPIO_WritePin(LCD_NSS_GPIO_Port, LCD_NSS_Pin, GPIO_PIN_RESET)

/****************************************************************************//**
 *
 * 								port function prototypes
 *
 *******************************************************************************/

void Sleep(uint32_t Delay);
unsigned long portGetTickCnt(void);

#define S1_SWITCH_ON  (1)
#define S1_SWITCH_OFF (0)
//when switch (S1) is 'on' the pin is low
int port_is_boot1_on(uint16_t x);
int port_is_switch_on(uint16_t GPIOpin);
int port_is_boot1_low(void);

void port_wakeup_dw1000(void);
void port_wakeup_dw1000_fast(void);

void port_set_dw1000_slowrate(void);
void port_set_dw1000_fastrate(void);

void process_dwRSTn_irq(void);
void process_deca_irq(void);

void led_on(led_t led);
void led_off(led_t led);

int  peripherals_init(void);
void spi_peripheral_init(void);

void setup_DW1000RSTnIRQ(int enable);

void reset_DW1000(void);


void port_LCD_RS_set(void);
void port_LCD_RS_clear(void);
void port_LCD_RW_set(void);
void port_LCD_RW_clear(void);

ITStatus EXTI_GetITEnStatus(uint32_t x);

uint32_t port_GetEXT_IRQStatus(void);
uint32_t port_CheckEXT_IRQ(void);
void port_DisableEXT_IRQ(void);
void port_EnableEXT_IRQ(void);
extern uint32_t		HAL_GetTick(void);
HAL_StatusTypeDef 	flush_report_buff(void);

#ifdef __cplusplus
}
#endif

#endif /* PORT_H_ */



/*
 * Taken from the Linux Kernel
 *
 */

#ifndef _LINUX_CIRC_BUF_H
#define _LINUX_CIRC_BUF_H 1

struct circ_buf {
	char *buf;
	int head;
	int tail;
};

/* Return count in buffer.  */
#define CIRC_CNT(head,tail,size) (((head) - (tail)) & ((size)-1))

/* Return space available, 0..size-1.  We always leave one free char
   as a completely full buffer has head == tail, which is the same as
   empty.  */
#define CIRC_SPACE(head,tail,size) CIRC_CNT((tail),((head)+1),(size))

/* Return count up to the end of the buffer.  Carefully avoid
   accessing head and tail more than once, so they can change
   underneath us without returning inconsistent results.  */
#define CIRC_CNT_TO_END(head,tail,size) \
	({int end = (size) - (tail); \
	  int n = ((head) + end) & ((size)-1); \
	  n < end ? n : end;})

/* Return space available up to the end of the buffer.  */
#define CIRC_SPACE_TO_END(head,tail,size) \
	({int end = (size) - 1 - (head); \
	  int n = (end + (tail)) & ((size)-1); \
	  n <= end ? n : end+1;})

#endif /* _LINUX_CIRC_BUF_H  */

