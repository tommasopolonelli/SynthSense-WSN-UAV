/*!
 * @file    TMP117.cpp
 * @author  Nils Minor
 * 
 * @license  GNU GENERAL PUBLIC LICENSE (see license.txt)
 * 
 * v1.0.0   - Initial library version
 * 
 * 
 */


#include "main.h"
#include "macro.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "gpio.h"
#include "TMP117.h"

void (*newDataCallback) (void);
int alert_type = NOALERT;


void      TMP117_lockEEPROM (void);
uint8_t      TMP117_EEPROMisBusy (void);
void      TMP117_unlockEEPROM (void);
void      TMP117_printConfig (uint16_t reg_value);
void      TMP117_writeConfig (uint16_t config_data);
uint16_t  TMP117_i2cRead2B (uint8_t reg);
void      TMP117_i2cWrite2B (uint8_t reg, uint16_t data);

/*!
    @brief   Constructor 
    @param   addr device I2C address [0x48 - 0x4B]
 */
/*TMP117_TMP117 (uint8_t addr) {
  Wire.begin();

  address = addr;
  alert_pin = -1;
  alert_type = NOALERT;
  newDataCallback = NULL;
}*/

/*!
    @brief   Initialize in default mode 
    @param   _newDataCallback   callback function will be called when new data is available
 */
uint8_t Init_TMP117 ( void (*_newDataCallback) (void) ) {

	/* check the device ID */
	if (TMP117_getDeviceID () != TMP117_DEV_ID){
		/* config failed */
		return 1;
	}

	TMP117_setConvMode (CONTINUOUS);
	//TMP117_setConvMode (SHUTDOWN);
	TMP117_setConvTime (C125mS);
	TMP117_setAveraging (AVE8);
	TMP117_setAlertMode (DATA);
	TMP117_setOffsetTemperature(0);

	newDataCallback = _newDataCallback;

	return 0;

}

/*!
    @brief    Read configuration register and handle events.
              Should be called in loop in order to call callback functions 
 */
void TMP117_update (void) {
	TMP117_readConfig ();
}

/*!
    @brief   Performs a soft reset. All default values will be loaded to the configuration register
 */
void TMP117_softReset ( void ) {
	uint16_t reg_value = 0;
	reg_value |= 1UL << 1;
	TMP117_writeConfig ( reg_value );
}

/*!
    @brief   Set alert pin mode 

    @param   mode TMP117_PMODE [Thermal-Alert-Data]
 */
void      TMP117_setAlertMode ( enum TMP117_PMODE mode) {
	uint16_t reg_value = TMP117_readConfig ();
	if (mode == THERMAL) {
		reg_value |= 1UL << 4;    // change to thermal mode
		reg_value &= ~(1UL << 2); // set pin as alert flag
		reg_value &= ~(1UL << 3); // alert pin low active
	}
	else if (mode == ALERT) {
		reg_value &= ~(1UL << 4); // change to alert mode
		reg_value &= ~(1UL << 2); // set pin as alert flag
		reg_value &= ~(1UL << 3); // alert pin low active
	}
	else if (mode == DATA) {
		reg_value |= 1UL << 2;    // set pin as data ready flag
	}
	TMP117_writeConfig ( reg_value );
}

/*!
    @brief   Set alert callback function and ISR pin
    @param   *allert_callback  callback function
    @param   pin callback pin (INT?)
 */
void      TMP117_setAllertCallback (void (*allert_callback)(void), uint8_t pin) {

	/* FALLING CALLBACK !!!  */
	//alert_pin = pin;
	//pinMode(pin, INPUT_PULLUP);
	//attachInterrupt(digitalPinToInterrupt(pin), allert_callback, FALLING ); // Sets up pin 2 to trigger "alert" ISR when pin changes H->L and L->H

}

/*!
    @brief    Set alert temperature

    @param    lowtemp   low boundary alert temperature
    @param    hightemp  high boundary alert temperature  
 */
void      TMP117_setAllertTemperature (double lowtemp, double hightemp) {

	uint16_t high_temp_value = hightemp / TMP117_RESOLUTION;
	uint16_t low_temp_value = lowtemp / TMP117_RESOLUTION;

	TMP117_i2cWrite2B (TMP117_REG_TEMP_HIGH_LIMIT , high_temp_value);
	TMP117_i2cWrite2B (TMP117_REG_TEMP_LOW_LIMIT , low_temp_value);
}

/*!
    @brief    Set conversion mode

    @param    cmode   _TMP117_CMODE [CONTINUOUS-SHUTDOWN-ONESHOT]
 */
void      TMP117_setConvMode ( enum TMP117_CMODE cmode) {
	uint16_t reg_value = TMP117_readConfig ();
	reg_value &= ~((1UL << 11) | (1UL << 10));       // clear bits
	reg_value = reg_value | ( cmode  & 0x03 ) << 10; // set bits
	TMP117_writeConfig ( reg_value );
}

/*!
    @brief    Set conversion time

    @param    convtime  _TMP117_CONVT [C15mS5-C125mS-C250mS-C500mS-C1S-C4S-C8S-C16S]
 */
void      TMP117_setConvTime ( enum TMP117_CONVT convtime ) {
	uint16_t reg_value = TMP117_readConfig ();
	reg_value &= ~((1UL << 9) | (1UL << 8) | (1UL << 7));       // clear bits
	reg_value = reg_value | ( convtime  & 0x07 ) << 7;          // set bits
	TMP117_writeConfig ( reg_value );
}
/*!
    @brief    Set averaging mode

    @param    ave  _TMP117_AVE [NOAVE-AVE8-AVE32-AVE64]
 */
void      TMP117_setAveraging ( enum TMP117_AVE ave ) {
	uint16_t reg_value = TMP117_readConfig ();
	reg_value &= ~((1UL << 6) | (1UL << 5) );       // clear bits
	reg_value = reg_value | ( ave & 0x03 ) << 5;          // set bits
	TMP117_writeConfig ( reg_value );
}

/*!
    @brief    Set offset temperature

    @param    double  target offset temperature  in the range of ±256°C  
 */
void      TMP117_setOffsetTemperature ( double offset ) {
	int16_t offset_temp_value = offset / TMP117_RESOLUTION;
	TMP117_i2cWrite2B (TMP117_REG_TEMPERATURE_OFFSET , offset_temp_value);
}

/*!
    @brief    Set target temperature for calibration purpose

    @param    double  target temperature to calibrate to in the range of ±256°C  
 */
void      TMP117_setTargetTemperature ( double target ) {
	double actual_temp = TMP117_getTemperature ( );
	double delta_temp =  target - actual_temp;
	TMP117_setOffsetTemperature ( delta_temp );
}

/*!
    @brief    Read configuration register and handle events.

    @return   uint16_t  read value of the configuration regsiter          
 */
uint16_t  TMP117_readConfig (void) {
	uint16_t reg_value = TMP117_i2cRead2B ( TMP117_REG_CONFIGURATION );
	//uint8_t high_alert = reg_value >> 15 & 1UL;
	//uint8_t low_alert = reg_value >> 14 & 1UL;
	uint8_t data_ready = reg_value >> 13 & 1UL;
	//uint8_t eeprom_busy = reg_value >> 12 & 1UL;

	if (data_ready && newDataCallback != NULL)
		newDataCallback ();

	if (reg_value >> 15 & 1UL) {
		alert_type = HIGHALERT;
	}
	else if (reg_value >> 14 & 1UL) {
		alert_type = LOWALERT;
	}
	else {
		alert_type = NOALERT;
	}

	//printConfig ( reg_value );

	return reg_value;
}

/*!
    @brief    Returns the recalculated temperature

    @return   double  temperature in °C
 */
double    TMP117_getTemperature (void) {
	int16_t temp = TMP117_i2cRead2B( TMP117_REG_TEMPERATURE );
	return  (temp * TMP117_RESOLUTION);
}
/*!
    @brief    Get Device Revision 

    @return   uint16_t device revision
 */
uint16_t  TMP117_getDeviceRev (void) {
	// read bits [15:12]
	uint16_t raw = TMP117_i2cRead2B( TMP117_REG_DEVICE_ID );

	return ( (raw >> 12) & 0x3);
}

/*!
    @brief    Get Device ID (always 0x117)

    @return   uint16_t  device ID
 */
uint16_t  TMP117_getDeviceID (void) {
	// read bits [11:0]
	uint16_t raw = TMP117_i2cRead2B( TMP117_REG_DEVICE_ID );
	return (raw & 0x0fff);
}

/*!
    @brief    Returns the information which alert type happend

    @return   TMP117_ALERT [NoAlert-HighTempAlert-LowTempAlert]
 */
enum TMP117_ALERT TMP117_getAlertType ( void ) {
	return alert_type;
}

/*!
    @brief    Returns the content of the offset register in °C

    @return   double  offset temperature in °C
 */
double    TMP117_getOffsetTemperature (void) {
	int16_t temp = TMP117_i2cRead2B( TMP117_REG_TEMPERATURE_OFFSET );
	return  (temp * TMP117_RESOLUTION);
}

/*!
    @brief    Write data to EEPROM register

    @param    data        data to write to the EEPROM

    @param    eeprom_nr   represents the EEPROM number [1 - 3] 
 */
void      TMP117_writeEEPROM (uint16_t data, uint8_t eeprom_nr) {
	if (!TMP117_EEPROMisBusy()) {
		TMP117_unlockEEPROM();
		switch (eeprom_nr) {
		case 1 : TMP117_i2cWrite2B ( TMP117_REG_EEPROM1, data); break;
		case 2 : TMP117_i2cWrite2B ( TMP117_REG_EEPROM2, data); break;
		case 3 : TMP117_i2cWrite2B ( TMP117_REG_EEPROM3, data); break;
		//default: Serial.println("EEPROM value must be between 1 and 3");
		}
		TMP117_lockEEPROM();
	}
	else {
		//Serial.println("EEPROM is busy");
	}
}

/*!
    @brief    Read data from EEPROM register

    @param    eeprom_nr  represents the EEPROM number [1 - 3] 

    @return   uint16_t   read EEPROM data
 */
uint16_t  TMP117_readEEPROM (uint8_t eeprom_nr) {
	// read the 48 bit number from the EEPROM
	if (!TMP117_EEPROMisBusy()) {
		uint16_t eeprom_data = 0;
		switch (eeprom_nr) {
		case 1 : eeprom_data = TMP117_i2cRead2B( TMP117_REG_EEPROM1 ); break;
		case 2 : eeprom_data = TMP117_i2cRead2B( TMP117_REG_EEPROM2 ); break;
		case 3 : eeprom_data = TMP117_i2cRead2B( TMP117_REG_EEPROM3 ); break;
//		default: Serial.println("EEPROM value must be between 1 and 3");
		}
		return eeprom_data;
	}
	else {
//		Serial.println("EEPROM is busy");
	}
	return (uint16_t)eeprom_nr;
}


/**************************************************************************/
/* ********************* Library internal functions  ******************** */
/**************************************************************************/

/*!
    @brief    Write two bytes (16 bits) to TMP117 I2C sensor

    @param    reg  target register
    @param    data data to write
 */
void      TMP117_i2cWrite2B (uint8_t reg, uint16_t data){
	/*Wire.beginTransmission(address);
	Wire.write( reg );
	Wire.write( (data>>8) );
	Wire.write( (data&0xff) );
	Wire.endTransmission( );*/
	uint8_t b[2];
	b[0] = (data>>8);
	b[1] = (data&0xff);
	if (HAL_I2C_Mem_Write
				(&hi2c1, (uint16_t) TMP117_I2C_ADDR, (uint16_t) reg, I2C_MEMADD_SIZE_8BIT, b, 2, 100)
				!= HAL_OK){
			return;
		}
	//osDelay(1);
}

/*!
    @brief    Read two bytes (16 bits) from TMP117 I2C sensor

    @param    reg  target register to read from

    @return   uint16_t  read data
 */
uint16_t  TMP117_i2cRead2B (uint8_t reg) {
	uint8_t data[2] = {0};
	int16_t datac = 0;

	/*Wire.beginTransmission(address);
	Wire.write(reg);
	Wire.endTransmission();
	Wire.requestFrom(address, 2);*/
	if (HAL_I2C_Mem_Read(
			&hi2c1, (uint16_t) TMP117_I2C_ADDR, (uint16_t) reg, I2C_MEMADD_SIZE_8BIT, data, 2, 100)
			!= HAL_OK){
		return 1;
	}

	datac = ((data[0] << 8) | data[1]);

	return datac;
}

/*!
    @brief    Write configuration to config register

    @param    config_data  configuration
 */
void      TMP117_writeConfig (uint16_t config_data) {
	TMP117_i2cWrite2B (TMP117_REG_CONFIGURATION, config_data);
}

/*!
    @brief    Prints configuration in user readable format

    @param    reg_value  configuration value
 */
void      TMP117_printConfig (uint16_t reg_value) {

/*	Serial.println(reg_value, BIN);

	Serial.println("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
	Serial.print ("HIGH alert:  ");
	Serial.println( ( reg_value >> 15) & 0b1 , BIN);
	Serial.print ("LOW alert:   ");
	Serial.println( ( reg_value >> 14) & 0b1 , BIN);
	Serial.print ("Data ready:  ");
	Serial.println( ( reg_value >> 13) & 0b1 , BIN);
	Serial.print ("EEPROM busy: ");
	Serial.println( ( reg_value >> 12) & 0b1 , BIN);
	Serial.print ("MOD[1:0]:    ");
	Serial.println( ( reg_value >> 10) & 0b11 , BIN);
	Serial.print ("CONV[2:0]:   ");
	Serial.println( ( reg_value >> 7)  & 0b111 , BIN);
	Serial.print ("AVG[1:0]:    ");
	Serial.println( ( reg_value >> 5)  & 0b11 , BIN);
	Serial.print ("T/nA:        ");
	Serial.println( ( reg_value >> 4) & 0b1 , BIN);
	Serial.print ("POL:         ");
	Serial.println( ( reg_value >> 3) & 0b1 , BIN);
	Serial.print ("DR/Alert:    ");
	Serial.println( ( reg_value >> 2) & 0b1 , BIN);
	Serial.print ("Soft_Reset:  ");
	Serial.println( ( reg_value >> 1) & 0b1 , BIN);

	*/
}
/*!
    @brief    Lock EEPROM, write protection
 */
void      TMP117_lockEEPROM (void) {
	// clear bit 15
	uint16_t code = 0;
	code &= ~(1UL << 15);
	TMP117_i2cWrite2B ( TMP117_REG_EEPROM_UNLOCK, code );
	osDelay(100);
}

/*!
    @brief    Unlock EEPROM, remove write protection
 */
void      TMP117_unlockEEPROM (void) {
	// set bit 15
	uint16_t code = 0;
	code |= 1UL << 15;
	TMP117_i2cWrite2B ( TMP117_REG_EEPROM_UNLOCK, code );
	osDelay(100);
}

/*!
    @brief    States if the EEPROM is busy

    @return   Ture if the EEPROM is busy, fals else
 */
uint8_t      TMP117_EEPROMisBusy (void) {
	// Bit 14 indicates the busy state of the eeprom
	uint16_t code = TMP117_i2cRead2B ( TMP117_REG_EEPROM_UNLOCK );
	return (uint8_t) ((code >> 14) & 0x01);
}


