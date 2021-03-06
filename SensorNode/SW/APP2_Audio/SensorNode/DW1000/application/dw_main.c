/*! ----------------------------------------------------------------------------
 *  @file    dw_main.c
 *  @brief   main loop for the DecaRanging application
 *
 * @attention
 *
 * Copyright 2016 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */
/* Includes */
#include "main.h"
#include "macro.h"
#include "cmsis_os.h"
#include "compiler.h"
#include "port.h"
#include "dma.h"

#include "instance.h"

#include "deca_types.h"
#include "Motion_Sensors.h"
#include "Flash_Mngm.h"

#include "deca_spi.h"
#include "stdio.h"

#include "IoT.h"


static void IoT_TX (uint16_t len, uint16_t msgflags, uint8 mode);
static uint8_t Enable_Transfer( void );
static uint8_t Disable_Transfer( void );
static void Data_Transfer_main( void );
static void IoT_Exit (void);


#define SWS1_TXSPECT_MODE	0x80  //Continuous TX spectrum mode
#define SWS1_ANC_MODE 		0x08  //anchor mode
#define SWS1_SHF_MODE		0x10  //short frame mode (6.81M) (switch S1-5)
#define SWS1_64M_MODE		0x20  //64M PRF mode (switch S1-6)
#define SWS1_CH5_MODE		0x40  //channel 5 mode (switch S1-7)

int dr_mode = 0;
int instance_mode = ANCHOR;

uint8 s1switch = 0;
int chan, tagaddr, ancaddr, prf;

//#define LCD_BUFF_LEN (100)
//uint8 dataseq[LCD_BUFF_LEN];
//uint8 dataseq1[LCD_BUFF_LEN];

int ranging = 0;


IoT_tx_stat_e IoT_stato;
srd_msg_dsss tx_msg;
uint8_t iot_mode = 0;
uint8_t n_retransmission;
uint8_t run;
uint32_t timer;

//Configuration for DecaRanging Modes (8 default use cases selectable by the switch S1 on EVK)
instanceConfig_t chConfig[8] ={
		//mode 1 - S1: 7 off, 6 off, 5 off
		{
				2,              // channel
				3,              // preambleCode
				DWT_PRF_16M,    // prf
				DWT_BR_110K,    // datarate
				DWT_PLEN_1024,  // preambleLength
				DWT_PAC32,      // pacSize
				1,       // non-standard SFD
				(1025 + 64 - 32) //SFD timeout
		},
		//mode 2
		{
				2,              // channel
				3,             // preambleCode
				DWT_PRF_16M,    // prf
				DWT_BR_6M8,    // datarate
				DWT_PLEN_128,   // preambleLength
				DWT_PAC8,       // pacSize
				0,       // non-standard SFD
				(129 + 8 - 8) //SFD timeout
		},
		//mode 3
		{
				2,              // channel
				9,             // preambleCode
				DWT_PRF_64M,    // prf
				DWT_BR_110K,    // datarate
				DWT_PLEN_1024,  // preambleLength
				DWT_PAC32,      // pacSize
				1,       // non-standard SFD
				(1025 + 64 - 32) //SFD timeout
		},
		//mode 4
		{
				2,              // channel
				9,             // preambleCode
				DWT_PRF_64M,    // prf
				DWT_BR_6M8,    // datarate
				DWT_PLEN_128,   // preambleLength
				DWT_PAC8,       // pacSize
				0,       // non-standard SFD
				(129 + 8 - 8) //SFD timeout
		},
		//mode 5
		{
				5,              // channel
				3,             // preambleCode
				DWT_PRF_16M,    // prf
				DWT_BR_110K,    // datarate
				DWT_PLEN_1024,  // preambleLength
				DWT_PAC32,      // pacSize
				1,       // non-standard SFD
				(1025 + 64 - 32) //SFD timeout
		},
		//mode 6
		{
				5,              // channel
				3,             // preambleCode
				DWT_PRF_16M,    // prf
				DWT_BR_6M8,    // datarate
				DWT_PLEN_128,   // preambleLength
				DWT_PAC8,       // pacSize
				0,       // non-standard SFD
				(129 + 8 - 8) //SFD timeout
		},
		//mode 7
		{
				5,              // channel
				9,             // preambleCode
				DWT_PRF_64M,    // prf
				DWT_BR_110K,    // datarate
				DWT_PLEN_1024,  // preambleLength
				DWT_PAC32,      // pacSize
				1,       // non-standard SFD
				(1025 + 64 - 32) //SFD timeout
		},
		//mode 8
		{
				5,              // channel
				9,             // preambleCode
				DWT_PRF_64M,    // prf
				DWT_BR_6M8,    // datarate
				DWT_PLEN_128,   // preambleLength
				DWT_PAC8,       // pacSize
				0,       // non-standard SFD
				(129 + 8 - 8) //SFD timeout
		}
};


uint32 inittestapplication(uint8 s1switch);


int decarangingmode(uint8 s1switch)
{
	int mode = 0;

	if(s1switch & SWS1_SHF_MODE)
	{
		mode = 1;
	}

	if(s1switch & SWS1_64M_MODE)
	{
		mode = mode + 2;
	}
	if(s1switch & SWS1_CH5_MODE)
	{
		mode = mode + 4;
	}

	return mode;
}

uint32 inittestapplication(uint8 s1switch)
{
	uint32 devID ;
	int result;

	port_set_dw1000_slowrate();  //max SPI before PLLs configured is ~4M

	//this is called here to wake up the device (i.e. if it was in sleep mode before the restart)
	devID = instancereaddeviceid() ;
	if(DWT_DEVICE_ID != devID) //if the read of device ID fails, the DW1000 could be asleep
	{
		port_wakeup_dw1000();

		devID = instancereaddeviceid() ;
		// SPI not working or Unsupported Device ID
		if(DWT_DEVICE_ID != devID)
			return(-1) ;
		//clear the sleep bit - so that after the hard reset below the DW does not go into sleep
		dwt_softreset();
	}

	//reset the DW1000 by driving the RSTn line low
	reset_DW1000();

	result = instance_init() ;
	if (0 > result) return(-1) ; // Some failure has occurred

	port_set_dw1000_fastrate();
	devID = instancereaddeviceid() ;

	if (DWT_DEVICE_ID != devID)   // Means it is NOT DW1000 device
	{
		// SPI not working or Unsupported Device ID
		return(-1) ;
	}

	if(s1switch & SWS1_ANC_MODE)
	{
		instance_mode = ANCHOR;

		led_on(LED_PC6);

	}
	else
	{
		instance_mode = TAG;
		led_on(LED_PC7);
	}

	instance_init_s(instance_mode);
	dr_mode = decarangingmode(s1switch);

	chan = chConfig[dr_mode].channelNumber ;
	prf = (chConfig[dr_mode].pulseRepFreq == DWT_PRF_16M)? 16 : 64 ;

	instance_config(&chConfig[dr_mode]) ;                  // Set operating channel etc

	instancesettagsleepdelay(POLL_SLEEP_DELAY, BLINK_SLEEP_DELAY); //set the Tag sleep time

	instance_init_timings();

	return devID;
}


/*
 * @fn configure_continuous_txspectrum_mode
 * @brief   test application for production to check the TX power in various modes
 **/
void configure_continuous_txspectrum_mode(uint8 s1switch)
{

	//sprintf((char*)&dataseq[0], "Conti TX %s:%d:%d ", (s1switch & SWS1_SHF_MODE) ? "S" : "L", chan, prf);
	//memcpy(dataseq, (const uint8 *) "Spectrum Test   ", 16);


	//configure DW1000 into Continuous TX mode
	instance_starttxtest(0x1000);
	//measure the power
	//Spectrum Analyser set:
	//FREQ to be channel default e.g. 3.9936 GHz for channel 2
	//SPAN to 1GHz
	//SWEEP TIME 1s
	//RBW and VBW 1MHz
	//measure channel power

	//user has to reset the board to exit mode
	while(1)
	{
		Sleep(2);
	}

}

/*
 * @fn      main()
 * @brief   main entry point
 **/
void dw_main (void const * argument)
{
	int i = 0;
	int toggle = 1;
	double range_result = 0;
	double avg_result = 0;
	int canSleep;

	led_off(LED_ALL); //turn off all the LEDs

	uint8_t start = (uint8_t)((uint8_t *)argument);

	/* Disable deep sleep */
	Mngm_DeepSleep_en (0);

	peripherals_init();
	reset_DW1000();

	port_DisableEXT_IRQ(); 	//disable DW1000 IRQ until we configure the application

	// SEE macro.h for configurations */
	s1switch = port_is_boot1_on(0) << 1 // is_switch_on(TA_SW1_2) << 2
			| port_is_switch_on(TA_SW1_3) << 2
			| port_is_switch_on(TA_SW1_4) << 3
			| port_is_switch_on(TA_SW1_5) << 4
			| port_is_switch_on(TA_SW1_6) << 5
			| port_is_switch_on(TA_SW1_7) << 6
			| port_is_switch_on(TA_SW1_8) << 7;


	led_off(LED_ALL);

	if(inittestapplication(s1switch) == (uint32)-1)
	{
		/* init fail */
		return; //error
	}

	//test EVB1000 - used in EVK1000 production
	if((s1switch & SWS1_TXSPECT_MODE) == SWS1_TXSPECT_MODE) //to test TX power
	{
		//this function does not return!
		configure_continuous_txspectrum_mode(s1switch);
	}

	if(s1switch & SWS1_ANC_MODE)
	{
		instance_mode = ANCHOR;
	}
	else
	{
		instance_mode = TAG;
	}

	/* task disabled */
	if (!start){
		dwt_configuresleep((DWT_PRESRV_SLEEP|DWT_CONFIG|DWT_TANDV), DWT_WAKE_WK|DWT_WAKE_CS|DWT_SLP_EN);
		dwt_entersleep();
		vTaskDelete( osThreadGetId () );
		return;
	}

	port_EnableEXT_IRQ();

	// main loop
	while(1)
	{
		if (iot_mode){
#ifdef IoT_MODE
			port_DisableEXT_IRQ(); 	//disable DW1000 IRQ until we configure the application
			inittestapplication(s1switch);
			port_EnableEXT_IRQ();
			Data_Transfer_main();
			canSleep = 0;
#endif
		}else{

#ifdef IoT_MODE

#endif

			instance_data_t* inst = instance_get_local_structure_ptr(0);
			canSleep = instance_run();

			//if delayed TX scheduled but did not happen after expected time then it has failed... (has to be < slot period)
			//if anchor just go into RX and wait for next message from tags/anchors
			//if tag handle as a timeout
			if((inst->monitor == 1) && ((portGetTickCnt() - inst->timeofTx) > inst->finalReplyDelay_ms))
			{
				inst->wait4ack = 0;

				if(instance_mode == TAG)
				{
					inst_processrxtimeout(inst);
				}
				else //if(instance_mode == ANCHOR)
				{
					dwt_forcetrxoff();	//this will clear all events
					//enable the RX
					inst->testAppState = TA_RXE_WAIT ;
				}
				inst->monitor = 0;
			}

			if(instancenewrange())
			{
				int /*n,*/ l = 0, /*txl = 0, rxl = 0,*/ aaddr, taddr, txa, rxa, rng, rng_raw;
				ranging = 1;
				//send the new range information to LCD and/or USB
				range_result = instance_get_idist();
				avg_result = instance_get_adist();
				//set_rangeresult(range_result);

				//sprintf((char*)&dataseq[1], "LAST: %4.2f m", range_result);

				//sprintf((char*)&dataseq1[1], "AVG8: %4.2f m", avg_result);

				l = instance_get_lcount();
				//txl = instance_get_txl();
				//rxl = instance_get_rxl();
				aaddr = instancenewrangeancadd();
				taddr = instancenewrangetagadd();
				txa =  instancetxantdly();
				rxa =  instancerxantdly();
				rng = (int) (range_result*1000);
				rng_raw = (int) (avg_result*1000);

				if(instance_mode == TAG)
				{
					//n = sprintf((char*)&dataseq[0], "ia%04x t%04x %08x %08x %04x %04x %04x t", aaddr, taddr, rng, rng_raw, l, txa, rxa);
				}
				else
				{
					//n = sprintf((char*)&dataseq[0], "ia%04x t%04x %08x %08x %04x %04x %04x a", aaddr, taddr, rng, rng_raw, l, txa, rxa);
				}

			}

			if(ranging == 0)
			{
				if(instance_mode != ANCHOR)
				{
					if(instancesleeping())
					{
						if(toggle)
						{
							toggle = 0;
							//memcpy(&dataseq[0], (const uint8 *) "    AWAITING    ", 16);
							//memcpy(&dataseq[0], (const uint8 *) "    RESPONSE    ", 16);
						}
						else
						{
							toggle = 1;
							//memcpy(&dataseq[2], (const uint8 *) "   TAG BLINK    ", 16);
							//sprintf((char*)&dataseq[0], "%llX", instance_get_addr());
						}
					}

					if(instanceanchorwaiting() == 2)
					{
						ranging = 1;
						//memcpy(&dataseq[0], (const uint8 *) "    RANGING WITH", 16);
						//sprintf((char*)&dataseq[0], "%016llX", instance_get_anchaddr());
					}
				}
				else //if(instance_mode == ANCHOR)
				{
					if(instanceanchorwaiting())
					{
						toggle+=2;

						if(toggle > 300000)
						{
							if(toggle & 0x1)
							{
								toggle = 0;
								//memcpy(&dataseq[0], (const uint8 *) "    AWAITING    ", 16);
								//memcpy(&dataseq[0], (const uint8 *) "      POLL      ", 16);
							}
							else
							{
								toggle = 1;
								//memcpy(&dataseq[0], (const uint8 *) " DISCOVERY MODE ", 16);
								//sprintf((char*)&dataseq[0], "%llX", instance_get_addr());
							}
						}

					}
					else if(instanceanchorwaiting() == 2)
					{
						//memcpy(&dataseq[0], (const uint8 *) "    RANGING WITH", 16);
						//sprintf((char*)&dataseq[0], "%llX", instance_get_tagaddr());
					}
				}
			}

		}

		CHECK_STACK_OF();

		if(canSleep){
			//__WFI();
			/* Enable deep sleep */
			Mngm_DeepSleep_en (1);
			/* Task Sleep */
			port_RTOS_WaitEvent();
			/* Disable deep sleep */
			Mngm_DeepSleep_en (0);
			//Sleep(1);
		}

	}


	return;
}


/*
 * @fn Data_Transfer_main
 * @brief Manage the transfer mode
 **/
static void Data_Transfer_main( void )
{
	uint8 command;
	event_data_t *evtdata;
	instance_data_t* inst = instance_get_local_structure_ptr(0);
	uint16_t len;

	IoT_stato = TX_IDLE;
	n_retransmission = 0;

	if (instance_mode == TAG){
		command = Enable_Transfer();
		if (command){
			/* host ack */
			//dataseq[0] = IOT_ENABLE;
		}else{
			//dataseq[0] = IOT_DISABLE;
			IoT_Exit();
			return;
		}
	}else{
		/* ANCHOR */
		/* Send the ACK */
		tx_msg.messageData[0] = IOT_ACK;
		IoT_TX (IOT_ACK_LEN, (FRAME_BEACON), 0);
		/* host ack */
		//dataseq[0] = IOT_ENABLE;
	}

	Sleep(10);

	/* Setup config */
	dwt_enableframefilter(DWT_FF_NOTYPE_EN); //disable frame filtering
	// First time anchor listens we don't do a delayed RX
	dwt_setrxaftertxdelay(0);
	dwt_setrxtimeout(0);

	// disable timer
	dwt_setrxtimeout((uint16)0);  //units are symbols
	//turn RX on
	instancerxon(inst, 0, 0) ;   // turn RX on, with/without delay
	/* clear previous events */
	instance_clearevents();
	IoT_stato = TX_IDLE;

	/* Disable APP Motion Sens */
	MotionSens_SendState();
	Sleep(10);

	run = 1;
	while(run){

		evtdata = instance_getevent(SIG_RX_UNKNOWN);

		if(evtdata->type == DWT_SIG_TX_DONE){
			/* next state */
			if(IoT_stato == TX_RUNNING){
				IoT_stato = TX_IDLE;
				dwt_setrxtimeout((uint16)0);  //units are symbols
				instancerxon(inst, 0, 0) ;   // turn RX on, with/without delay
			}else{
				/*  automatic RX after TX */
			}
		}
		if (evtdata->type == DWT_SIG_RX_TIMEOUT){
			/* next state */
			if((IoT_stato == WAIT_ACK) /*&& (n_retransmission < MAX_N_TX)*/){
				// disable timer
				dwt_setrxtimeout((uint16)0);  //units are symbols
				//turn RX on
				instancerxon(inst, 0, 0) ;   // turn RX on, with/without delay
				/* TX Failed */
				IoT_stato = TX_IDLE;
				/* host ack */
				//dataseq[0] = IOT_NACK;
				//TODO
				//send_usbmessage(&dataseq[0], IOT_NACK_LEN);
			}else{
				/* unexpected event in RX */
				/* restart the RX */
				dwt_forcetrxoff();
				dwt_rxreset();
				dwt_setrxtimeout((uint16)0);  //units are symbols
				//turn RX on
				instancerxon(inst, 0, 0) ;   // turn RX on, with/without delay
			}
		}
		if((evtdata->type == DWT_SIG_RX_BLINK) || (evtdata->type == DWT_SIG_RX_OKAY)){
			/* unexpected event in RX */
			/* restart the RX */
			dwt_forcetrxoff();
			dwt_rxreset();
			dwt_setrxtimeout((uint16)0);  //units are symbols
			//turn RX on
			instancerxon(inst, 0, 0) ;   // turn RX on, with/without delay
			/* TX Failed */
			IoT_stato = TX_IDLE;
		}
		if (evtdata->type == SIG_RX_UNKNOWN){
			if (IoT_stato == TX_IDLE){
				/* check if an ACK is required */
				if (CHECK_ACK_REQ(evtdata->msgu.rxmsg_ss.frameCtrl)){
					dwt_forcetrxoff();
					/* ACK requested */
					tx_msg.messageData[0] = IOT_ACK;
					IoT_TX (IOT_ACK_LEN, (FRAME_BEACON), 0);
				}
				/* tx on USB */
				if(IS_DATA(evtdata->msgu.rxmsg_ss.messageData)){
					/* remove header len */
					len = evtdata->rxLength - FRAME_CONTROL_BYTES - FRAME_SEQ_NUM_BYTES - FRAME_PANID -
							ADDR_BYTE_SIZE_S - ADDR_BYTE_SIZE_S - FRAME_CRC;
					//TODO
					//send_usbmessage(&evtdata->msgu.rxmsg_ss.messageData[0], len);
				}
				/* disable */
				if (IS_DISABLE(evtdata->msgu.rxmsg_ss.messageData)){
					//dataseq[0] = IOT_DISABLE;
					//send_usbmessage(&dataseq[0], IOT_DISABLE_LEN);
					run = 0;
				}
				/* check if an ACK has been sent */
				if (IoT_stato == TX_IDLE){
					// disable timer
					dwt_setrxtimeout((uint16)0);  //units are symbols
					//turn RX on
					instancerxon(inst, 0, 0) ;   // turn RX on, with/without delay
				}
			}else{
				if(IS_ACK(evtdata->msgu.rxmsg_ss.messageData)){
					// disable timer
					dwt_setrxtimeout((uint16)0);  //units are symbols
					//turn RX on
					instancerxon(inst, 0, 0) ;   // turn RX on, with/without delay
					/* TX ok */
					n_retransmission=0;
					IoT_stato = TX_IDLE;
					/* host ack */
					//dataseq[0] = IOT_ACK;
					//send_usbmessage(&dataseq[0], IOT_ACK_LEN);
				}else{
					// disable timer
					dwt_setrxtimeout((uint16)0);  //units are symbols
					//turn RX on
					instancerxon(inst, 0, 0) ;   // turn RX on, with/without delay
					/* TX Failed */
					n_retransmission=0;
					IoT_stato = TX_IDLE;
					/* host ack */
					//dataseq[0] = IOT_NACK;
					//send_usbmessage(&dataseq[0], IOT_NACK_LEN);
				}
			}
		}

		/* DATA TRANSMISSION ON MCU REQUEST */
		if(IoT_stato == TX_IDLE){
			dwt_forcetrxoff();

			/* ask for unconfirmed TX */
			tx_msg.messageData[0] = IOT_DATA_UNC;
			/* collect data from Int_Sensor_task */
			len = Int_Sensors_Stream (&tx_msg.messageData[1], 500);

			Sleep(5);

			if (len != 0){
				/* stream new packet */
				IoT_TX (len + IOT_DATA_LEN, (FRAME_BEACON ), 0);
			}else{
				/* disable transfer */
				if (instance_mode != TAG){
					/* disable tag */
					Disable_Transfer();
				}
				run = 0;
			}
		}

#if 0
		if((local_buff_length > 0) && (IoT_stato == TX_IDLE)){
			/* maximum length */
			if (local_buff_length < MAX_USER_PAYLOAD_STRING_SS){

				if (*local_buff_p == IOT_DATA){
					dwt_forcetrxoff();
					memcpy_DMA ((uint32_t *)local_buff_p,(uint32_t *)&tx_msg.messageData[0],local_buff_length);
					//memcpy(&tx_msg.messageData[0],local_buff_p,local_buff_length);
					IoT_TX (local_buff_length + IOT_DATA_LEN, (FRAME_BEACON | DEF_MAC_ACK),
							DWT_RESPONSE_EXPECTED);

				}else if (*local_buff_p == IOT_DATA_UNC){
					dwt_forcetrxoff();
					memcpy_DMA ((uint32_t *)local_buff_p,(uint32_t *)&tx_msg.messageData[0],local_buff_length);
					//memcpy(&tx_msg.messageData[0],local_buff_p,local_buff_length);
					IoT_TX (local_buff_length + IOT_DATA_LEN, (FRAME_BEACON ), 0);

				}else if (*local_buff_p == IOT_DISABLE){
					dataseq[0] = IOT_DISABLE;
					send_usbmessage(&dataseq[0], IOT_DISABLE_LEN);
					run = 0;
				}
			}

			local_buff_length = 0;
		}
#endif

	}

	/* disable anchor */
	if (instance_mode == TAG){
		Disable_Transfer();
	}else{
		dwt_forcetrxoff();
		dwt_rxreset();
		dwt_enableframefilter(DWT_FF_NOTYPE_EN); //disable frame filtering
		ranging = 0;
	}

	IoT_Exit();

	/* Enable APP Motion Sens */
	MotionSens_SendState();

}


/*
 * @fn Data_Transfer_main
 * @brief Manage the transfer mode
 **/
static uint8_t Enable_Transfer( void )
{
	uint8 counter=0;
	event_data_t *evtdata;

	dwt_forcetrxoff();
	dwt_rxreset();
	dwt_enableframefilter(DWT_FF_NOTYPE_EN); //disable frame filtering

	tx_msg.messageData[0] = IOT_ENABLE;
	IoT_TX (IOT_ENABLE_LEN, (FRAME_BEACON | DEF_MAC_ACK), DWT_RESPONSE_EXPECTED);

	/* start the IoT mode in the sensor */
	while(counter < MAX_N_TX){
		evtdata = instance_getevent(SIG_RX_UNKNOWN);
		/* start IoT procedure and wait ack */
		if (evtdata->type == DWT_SIG_RX_TIMEOUT){
			instance_clearevents();
			/* no ack, TX again */
			IoT_TX (IOT_ENABLE_LEN, (FRAME_BEACON | DEF_MAC_ACK), DWT_RESPONSE_EXPECTED);
			counter++;
		}else if (evtdata->type == SIG_RX_UNKNOWN){
			if (CHECK_FRAME_TYPE(evtdata->msgu.rxmsg_ss.frameCtrl,FRAME_BEACON)){
				/* ack ok */
				return 1;
			}else{
				/* no ack, TX again */
				IoT_TX (IOT_ENABLE_LEN, (FRAME_BEACON | DEF_MAC_ACK), DWT_RESPONSE_EXPECTED);
				counter++;
			}
		}

	}

	return 0;

}


/*
 * @fn Send a message
 * @brief Manage the TX
 **/
static void IoT_TX (uint16_t len, uint16_t msgflags, uint8 mode){

	static uint8_t count = 0;

	count++;
	if (mode == DWT_RESPONSE_EXPECTED){
		/* ACK expected */
		IoT_stato = WAIT_ACK;
		dwt_setrxtimeout((uint16)TXTIMEOUT);  //units are symbols
	}else{
		IoT_stato = TX_RUNNING;
	}

	/* msb first */
	len = len + FRAME_CONTROL_BYTES + FRAME_SEQ_NUM_BYTES + FRAME_PANID +
			ADDR_BYTE_SIZE_S + ADDR_BYTE_SIZE_S + FRAME_CRC;

	*((uint16_t *)tx_msg.destAddr) = FRAME_ADDR;
	*((uint16_t *)tx_msg.sourceAddr) = FRAME_ADDR;
	*((uint16_t *)tx_msg.frameCtrl) = DEF_MAC_HEADER | msgflags;
	tx_msg.seqNum = count;

	dwt_writetxdata(len, (uint8 *)(&tx_msg), 0) ;
	dwt_writetxfctrl(len, 0, 0);

	//to start the first frame - set TXSTRT
	dwt_starttx((DWT_START_TX_IMMEDIATE | mode)); //always using immediate TX and enable dealyed RX

}


/*
 * @fn Data_Transfer_main
 * @brief Manage the transfer mode
 **/
static uint8_t Disable_Transfer( void )
{
	uint8 counter=0;
	event_data_t *evtdata;

	dwt_forcetrxoff();
	dwt_rxreset();
	dwt_enableframefilter(DWT_FF_NOTYPE_EN); //disable frame filtering

	tx_msg.messageData[0] = IOT_DISABLE;
	IoT_TX (IOT_ENABLE_LEN, (FRAME_BEACON | DEF_MAC_ACK), DWT_RESPONSE_EXPECTED);

	/* start the IoT mode in the sensor */
	while(counter < MAX_N_TX){
		evtdata = instance_getevent(SIG_RX_UNKNOWN);
		/* start IoT procedure and wait ack */
		if (evtdata->type == DWT_SIG_RX_TIMEOUT){
			instance_clearevents();
			/* no ack, TX again */
			IoT_TX (IOT_ENABLE_LEN, (FRAME_BEACON | DEF_MAC_ACK), DWT_RESPONSE_EXPECTED);
			counter++;
		}else if (evtdata->type == SIG_RX_UNKNOWN){
			if (CHECK_FRAME_TYPE(evtdata->msgu.rxmsg_ss.frameCtrl,FRAME_BEACON)){
				/* ack ok */
				return 1;
			}else{
				/* no ack, TX again */
				IoT_TX (IOT_ENABLE_LEN, (FRAME_BEACON | DEF_MAC_ACK), DWT_RESPONSE_EXPECTED);
				counter++;
			}
		}

	}

	return 0;

}

/*
 * @fn IoT_Exit
 * @brief Restart the ranging protocol
 **/
static void IoT_Exit (void){

	/* exit */
	iot_mode = 0;

	instance_clear();
	port_DisableEXT_IRQ(); 	//disable DW1000 IRQ until we configure the application
	reset_DW1000();
	inittestapplication(s1switch);
	port_EnableEXT_IRQ();

	/* reset all */
	//HAL_NVIC_SystemReset();

}


/*
 * @fn memcpy_DMA
 * @brief Copy between two memory location with DMA
 **/
extern DMA_HandleTypeDef hdma_memtomem_dma1_channel7;
#pragma GCC push_options
#pragma GCC optimize ("-O3")
void memcpy_DMA (uint32_t *SrcAddress, uint32_t *DstAddress, uint32_t DataLength){
	if (DataLength < 10){
		memcpy(DstAddress,SrcAddress,DataLength);
	}else{
		while(HAL_DMA_GetState(&hdma_memtomem_dma1_channel7) == HAL_DMA_STATE_BUSY);
		/* errors */
		if ((HAL_DMA_GetState(&hdma_memtomem_dma1_channel7) == HAL_DMA_STATE_TIMEOUT) ||
				(HAL_DMA_GetState(&hdma_memtomem_dma1_channel7) == HAL_DMA_STATE_TIMEOUT)){
			HAL_DMA_Abort(&hdma_memtomem_dma1_channel7);
		}
		/* stant new transfer */
		HAL_DMA_Start_IT(&hdma_memtomem_dma1_channel7,(uint32_t)SrcAddress,(uint32_t)DstAddress,DataLength);
	}
}
#pragma GCC pop_options

#pragma GCC push_options
#pragma GCC optimize ("-O3")
void memcpy_DMA_poll (void){

	/* check if the DMA is still working */
	//HAL_DMA_PollForTransfer(&hdma_memtomem_dma1_channel1, HAL_DMA_FULL_TRANSFER, 10);
	while(HAL_DMA_GetState(&hdma_memtomem_dma1_channel7) == HAL_DMA_STATE_BUSY);
	/* errors */
	if ((HAL_DMA_GetState(&hdma_memtomem_dma1_channel7) == HAL_DMA_STATE_TIMEOUT) ||
			(HAL_DMA_GetState(&hdma_memtomem_dma1_channel7) == HAL_DMA_STATE_TIMEOUT)){
		HAL_DMA_Abort(&hdma_memtomem_dma1_channel7);
	}

}
#pragma GCC pop_options



