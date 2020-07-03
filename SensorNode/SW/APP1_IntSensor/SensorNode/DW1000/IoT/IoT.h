/*! ----------------------------------------------------------------------------
 *  @file    IoT.h
 *  @brief   main loop for the IoT application
 *
 * @attention
 *
 * Copyright 2016 (c) DecaWave Ltd, Dublin, Ireland.
 *
 *
 * @author Tommaso Polonelli
 */

#ifndef IOT_IOT_H_
#define IOT_IOT_H_

/*! ----------------------------------------------------------------------------
 *  LCD
 */
#define RET_CURS_HOME 							(0x02)
#define LCD_MSG_1								"     IoT Mode   "
#define LCD_MSG_2								"Imperial College"
#define LCD_MSG_LEN								(16)

/*! ----------------------------------------------------------------------------
 *  MAC HEADER
 */
#define DEF_MAC_HEADER							0x8840

#define FRAME_BEACON							0x0000
#define FRAME_DATA								0x0001
#define FRAME_ACK								0x0002
#define FRAME_MAC								0x0003

#define DEF_MAC_ACK								0x0020
#define DEF_MAC_PEND							0x0010

#define FRAME_PAN_ID							0xdeca
#define FRAME_ADDR								0xFFFF

#define CHECK_FRAME_TYPE(x,y)					((x[0] & 0x03) == y)
#define CHECK_ACK_REQ(x)						(x[0] & DEF_MAC_ACK)
#define IS_DATA(x)								((x[0] == IOT_DATA) || (x[0] == IOT_DATA_UNC))
#define IS_ENABLE(x)							(x[0] == IOT_ENABLE)
#define IS_DISABLE(x)							(x[0] == IOT_DISABLE)
#define IS_ACK(x)								(x[0] == IOT_ACK)


/*! ----------------------------------------------------------------------------
 *  ENABLE TRANSFER
 */
typedef enum{
	TX_IDLE = 0,
	WAIT_ACK,
	TX_RUNNING,
	TX_SEND
}IoT_tx_stat_e;

#define IOT_ENABLE								'E'
#define IOT_ENABLE_LEN							(1)
#define IOT_DISABLE								'N'
#define IOT_DISABLE_LEN							(1)
#define IOT_ACK									'A'
#define IOT_ACK_LEN								(1)
#define IOT_NACK								'F'
#define IOT_NACK_LEN							(1)
#define IOT_DATA								'D'
#define IOT_DATA_LEN							(1)
#define IOT_DATA_UNC							'U'
#define IOT_DATA_LEN							(1)
#define RANGE_START								'S'
#define RANGE_START_LEN							(1)

#define MAX_N_TX								(3)
#define TXTIMEOUT								(50000)

extern uint8_t iot_mode;
extern uint8_t run;

#endif /* IOT_IOT_H_ */
