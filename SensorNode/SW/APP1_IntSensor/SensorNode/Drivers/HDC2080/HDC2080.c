/*
	HDC2080.cpp
	HDC2010.cpp originally created by: Brandon Fisher, August 1st 2017
	
	This code is release AS-IS into the public domain, no guarantee or warranty is given.
	
	Description: This library facilitates communication with, and configuration of,
	the HDC2080 Temperature and Humidity HDC2080_ It makes extensive use of the
	Wire.H library, and should be useable with both Arduino and Energia. 
*/

#include "main.h"
#include "macro.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "gpio.h"
#include "HDC2080.h"

int _addr; 									// Address of sensor
void openReg(uint8_t reg); 	    			// Points to a given register
uint8_t HDC2080_readReg(uint8_t reg);				// Reads a given register, returns 1 byte
void HDC2080_writeReg(uint8_t reg, uint8_t data); 	// Writes a byte of data to one register


//Define Register Map
	#define TEMP_LOW 0x00
	#define TEMP_HIGH 0x01
	#define HUMID_LOW 0x02
	#define HUMID_HIGH 0x03
	#define INTERRUPT_DRDY 0x04
	#define TEMP_MAX 0x05
	#define HUMID_MAX 0x06
	#define INTERRUPT_CONFIG 0x07
	#define TEMP_OFFSET_ADJUST 0x08
	#define HUM_OFFSET_ADJUST 0x09
	#define TEMP_THR_L 0x0A
	#define TEMP_THR_H 0x0B
	#define HUMID_THR_L 0x0C
	#define HUMID_THR_H 0x0D
	#define CONFIG 0x0E
	#define MEASUREMENT_CONFIG 0x0F
	#define MID_L 0xFC
	#define MID_H 0xFD
	#define DEVICE_ID_L 0xFE
	#define DEVICE_ID_H 0xFF
	
void HDC2080_HDC2080(uint8_t addr)
{
  _addr = addr;
}

void HDC2080_begin(void)
{
	/*Wire.begin();*/
}

float HDC2080_readTemp(void)
{
	uint8_t byte[2];
	uint16_t temp;
	byte[0] = HDC2080_readReg(TEMP_LOW);
	byte[1] = HDC2080_readReg(TEMP_HIGH);
	
	temp = (unsigned int)byte[1] << 8 | byte[0];
	
	return (float)(temp) * 165 / 65536 - 40;
	
}

float HDC2080_readHumidity(void)
{
	uint8_t byte[2];
	uint16_t humidity;
	byte[0] = HDC2080_readReg(HUMID_LOW);
	byte[1] = HDC2080_readReg(HUMID_HIGH);
	
	humidity = (unsigned int)byte[1] << 8 | byte[0];
	
	return (float)(humidity)/( 65536 )* 100;
	
}

void HDC2080_enableHeater(void)
{
	uint8_t configContents;	//Stores current contents of config register
	
	configContents = HDC2080_readReg(CONFIG);
	
	//set bit 3 to 1 to enable heater
	configContents = (configContents | 0x08);
	
	HDC2080_writeReg(CONFIG, configContents);
	
}

void HDC2080_disableHeater(void)
{
	uint8_t configContents;	//Stores current contents of config register
	
	configContents = HDC2080_readReg(CONFIG);
	
	//set bit 3 to 0 to disable heater (all other bits 1)
	configContents = (configContents & 0xF7);
	HDC2080_writeReg(CONFIG, configContents);
	
}

void HDC2080_openReg(uint8_t reg)
{
  /*Wire.beginTransmission(_addr); 		// Connect to HDC2080
  Wire.write(reg); 						// point to specified register
  Wire.endTransmission(); 				// Relinquish bus control*/
}

uint8_t HDC2080_readReg(uint8_t reg)
{
	/*openReg(reg);
	uint8_t reading; 					// holds byte of read data
	Wire.requestFrom(_addr, 1); 		// Request 1 byte from open register
	Wire.endTransmission();				// Relinquish bus control
	
	if (1 <= Wire.available())
	{
		reading = (Wire.read());			// Read byte
	}*/

	uint8_t reading;

	if (HAL_I2C_Mem_Read(
			&hi2c1, (uint16_t) HDC2080_I2C_ADDR, (uint16_t) reg, I2C_MEMADD_SIZE_8BIT, &reading, 1, 100)
			!= HAL_OK){
		return 1;
	}
	
	return reading;
}

void HDC2080_writeReg(uint8_t reg, uint8_t data)
{
	
  /*Wire.beginTransmission(_addr);		// Open Device
  Wire.write(reg);						// Point to register
  Wire.write(data);						// Write data to register 
  Wire.endTransmission();				// Relinquish bus control*/

	if (HAL_I2C_Mem_Write
			(&hi2c1, (uint16_t) HDC2080_I2C_ADDR, (uint16_t) reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100)
			!= HAL_OK){
		return;
	}
  
}

void HDC2080_setLowTemp(float temp)
{
	uint8_t temp_thresh_low;
	
	// Verify user is not trying to set value outside bounds
	if (temp < -40)
	{
		temp = -40;
	}
	else if (temp > 125)
	{
		temp = 125;
	}
	
	// Calculate value to load into register
	temp_thresh_low = (uint8_t)(256 * (temp + 40)/165);
	
	HDC2080_writeReg(TEMP_THR_L, temp_thresh_low);
	
}

void HDC2080_setHighTemp(float temp)
{ 
	uint8_t temp_thresh_high;
	
	// Verify user is not trying to set value outside bounds
	if (temp < -40)
	{
		temp = -40;
	}
	else if (temp > 125)
	{
		temp = 125;
	}
	
	// Calculate value to load into register
	temp_thresh_high = (uint8_t)(256 * (temp + 40)/165);
	
	HDC2080_writeReg(TEMP_THR_H, temp_thresh_high);
	
}

void HDC2080_setHighHumidity(float humid)
{
	uint8_t humid_thresh;
	
	// Verify user is not trying to set value outside bounds
	if (humid < 0)
	{
		humid = 0;
	}
	else if (humid > 100)
	{
		humid = 100;
	}
	
	// Calculate value to load into register
	humid_thresh = (uint8_t)(256 * (humid)/100);
	
	HDC2080_writeReg(HUMID_THR_H, humid_thresh);
	
}

void HDC2080_setLowHumidity(float humid)
{
	uint8_t humid_thresh;
	
	// Verify user is not trying to set value outside bounds
	if (humid < 0)
	{
		humid = 0;
	}
	else if (humid > 100)
	{
		humid = 100;
	}
	
	// Calculate value to load into register
	humid_thresh = (uint8_t)(256 * (humid)/100);
	
	HDC2080_writeReg(HUMID_THR_L, humid_thresh);
	
}

//  Return humidity from the low threshold register
float HDC2080_readLowHumidityThreshold(void)
{
	uint8_t regContents;
	
	regContents = HDC2080_readReg(HUMID_THR_L);
	
	return (float)regContents * 100/256;
	
}

//  Return humidity from the high threshold register
float HDC2080_readHighHumidityThreshold(void)
{
	uint8_t regContents;
	
	regContents = HDC2080_readReg(HUMID_THR_H);
	
	return (float)regContents * 100/256;
	
}

//  Return temperature from the low threshold register
float HDC2080_readLowTempThreshold(void)
{
	uint8_t regContents;
	
	regContents = HDC2080_readReg(TEMP_THR_L);
	
	return (float)regContents * 165/256 - 40;
	
}

//  Return temperature from the high threshold register
float HDC2080_readHighTempThreshold(void)
{
	uint8_t regContents;
	
	regContents = HDC2080_readReg(TEMP_THR_H);
	
	return (float)regContents * 165/256 - 40;
	
}


/* Upper two bits of the MEASUREMENT_CONFIG register controls
   the temperature resolution*/
void HDC2080_setTempRes(int resolution)
{ 
	uint8_t configContents;
	configContents = HDC2080_readReg(MEASUREMENT_CONFIG);
	
	switch(resolution)
	{
		case FOURTEEN_BIT:
			configContents = (configContents & 0x3F);
			break;
			
		case ELEVEN_BIT:
			configContents = (configContents & 0x7F);
			configContents = (configContents | 0x40);  
			break;
			
		case NINE_BIT:
			configContents = (configContents & 0xBF);
			configContents = (configContents | 0x80); 
			break;
			
		default:
			configContents = (configContents & 0x3F);
	}
	
	HDC2080_writeReg(MEASUREMENT_CONFIG, configContents);
	
}
/*  Bits 5 and 6 of the MEASUREMENT_CONFIG register controls
    the humidity resolution*/
void HDC2080_setHumidRes(int resolution)
{ 
	uint8_t configContents;
	configContents = HDC2080_readReg(MEASUREMENT_CONFIG);
	
	switch(resolution)
	{
		case FOURTEEN_BIT:
			configContents = (configContents & 0xCF);
			break;
			
		case ELEVEN_BIT:
			configContents = (configContents & 0xDF);
			configContents = (configContents | 0x10);  
			break;
			
		case NINE_BIT:
			configContents = (configContents & 0xEF);
			configContents = (configContents | 0x20); 
			break;
			
		default:
			configContents = (configContents & 0xCF);
	}
	
	HDC2080_writeReg(MEASUREMENT_CONFIG, configContents);
}

/*  Bits 2 and 1 of the MEASUREMENT_CONFIG register controls
    the measurement mode  */
void HDC2080_setMeasurementMode(int mode)
{ 
	uint8_t configContents;
	configContents = HDC2080_readReg(MEASUREMENT_CONFIG);
	
	switch(mode)
	{
		case TEMP_AND_HUMID:
			configContents = (configContents & 0xF9);
			break;
			
		case TEMP_ONLY:
			configContents = (configContents & 0xFC);
			configContents = (configContents | 0x02);  
			break;
			
		case HUMID_ONLY:
			configContents = (configContents & 0xFD);
			configContents = (configContents | 0x04); 
			break;
			
		default:
			configContents = (configContents & 0xF9);
	}
	
	HDC2080_writeReg(MEASUREMENT_CONFIG, configContents);
}

/*  Bit 0 of the MEASUREMENT_CONFIG register can be used
    to trigger measurements  */
void HDC2080_triggerMeasurement(void)
{ 
	uint8_t configContents;
	configContents = HDC2080_readReg(MEASUREMENT_CONFIG);

	configContents = (configContents | 0x01);
	HDC2080_writeReg(MEASUREMENT_CONFIG, configContents);
}

/*  Bit 7 of the CONFIG register can be used to trigger a 
    soft reset  */
void HDC2080_reset(void)
{
	uint8_t configContents;
	configContents = HDC2080_readReg(CONFIG);

	configContents = (configContents | 0x80);
	HDC2080_writeReg(CONFIG, configContents);
	osDelay(50);
}

/*  Bit 2 of the CONFIG register can be used to enable/disable 
    the interrupt pin  */
void HDC2080_enableInterrupt(void)
{
	uint8_t configContents;
	configContents = HDC2080_readReg(CONFIG);

	configContents = (configContents | 0x04);
	HDC2080_writeReg(CONFIG, configContents);
}

/*  Bit 2 of the CONFIG register can be used to enable/disable 
    the interrupt pin  */
void HDC2080_disableInterrupt(void)
{
	uint8_t configContents;
	configContents = HDC2080_readReg(CONFIG);

	configContents = (configContents & 0xFB);
	HDC2080_writeReg(CONFIG, configContents);
}


/*  Bits 6-4  of the CONFIG register controls the measurement 
    rate  */
void HDC2080_setRate(int rate)
{ 
	uint8_t configContents;
	configContents = HDC2080_readReg(CONFIG);
	
	switch(rate)
	{
		case MANUAL:
			configContents = (configContents & 0x8F);
			break;
			
		case TWO_MINS:
			configContents = (configContents & 0x9F);
			configContents = (configContents | 0x10);  
			break;
			
		case ONE_MINS:
			configContents = (configContents & 0xAF);
			configContents = (configContents | 0x20); 
			break;
		
		case TEN_SECONDS:
			configContents = (configContents & 0xBF);
			configContents = (configContents | 0x30); 
			break;
		
		case FIVE_SECONDS:
			configContents = (configContents & 0xCF);
			configContents = (configContents | 0x40); 
			break;
		
		case ONE_HZ:
			configContents = (configContents & 0xDF);
			configContents = (configContents | 0x50); 
			break;
		
		case TWO_HZ:
			configContents = (configContents & 0xEF);
			configContents = (configContents | 0x60); 
			break;
		
		case FIVE_HZ:
			configContents = (configContents | 0x70); 
			break;
			
		default:
			configContents = (configContents & 0x8F);
	}
	
	HDC2080_writeReg(CONFIG, configContents);
}

/*  Bit 1 of the CONFIG register can be used to control the  
    the interrupt pins polarity */
void HDC2080_setInterruptPolarity(int polarity)
{
	uint8_t configContents;
	configContents = HDC2080_readReg(CONFIG);
	
	switch(polarity)
	{
		case ACTIVE_LOW:
			configContents = (configContents & 0xFD);
			break;
			
		case ACTIVE_HIGH:
			configContents = (configContents | 0x02);  
			break;
			
		default:
			configContents = (configContents & 0xFD);
	}
	
	HDC2080_writeReg(CONFIG, configContents);
}

/*  Bit 0 of the CONFIG register can be used to control the  
    the interrupt pin's mode */
void HDC2080_setInterruptMode(int mode)
{
	uint8_t configContents;
	configContents = HDC2080_readReg(CONFIG);
	
	switch(mode)
	{
		case LEVEL_MODE:
			configContents = (configContents & 0xFE);
			break;
			
		case COMPARATOR_MODE:
			configContents = (configContents | 0x01);  
			break;
			
		default:
			configContents = (configContents & 0xFE);
	}
	
	HDC2080_writeReg(CONFIG, configContents);
}


uint8_t HDC2080_readInterruptStatus(void)
{
	uint8_t regContents;
	regContents = HDC2080_readReg(INTERRUPT_DRDY);
	return regContents;
	
}

//  Clears the maximum temperature register
void HDC2080_clearMaxTemp(void)
{ 
	HDC2080_writeReg(TEMP_MAX, 0x00);
}

//  Clears the maximum humidity register
void HDC2080_clearMaxHumidity(void)
{ 
	HDC2080_writeReg(HUMID_MAX, 0x00);
}

//  Reads the maximum temperature register
float HDC2080_readMaxTemp(void)
{
	uint8_t regContents;
	
	regContents = HDC2080_readReg(TEMP_MAX);
	
	return (float)regContents * 165/256 - 40;
	
}

//  Reads the maximum humidity register
float HDC2080_readMaxHumidity(void)
{
	uint8_t regContents;
	
	regContents = HDC2080_readReg(HUMID_MAX);
	
	return (float)regContents /256 * 100;
	
}


// Enables the interrupt pin for comfort zone operation
void HDC2080_enableThresholdInterrupt(void)
{
	
	uint8_t regContents;
	regContents = HDC2080_readReg(INTERRUPT_CONFIG);

	regContents = (regContents | 0x78);

	HDC2080_writeReg(INTERRUPT_CONFIG, regContents);
}

// Disables the interrupt pin for comfort zone operation
void HDC2080_disableThresholdInterrupt(void)
{
	uint8_t regContents;
	regContents = HDC2080_readReg(INTERRUPT_CONFIG);

	regContents = (regContents & 0x87);

	HDC2080_writeReg(INTERRUPT_CONFIG, regContents);
}

// enables the interrupt pin for DRDY operation
void HDC2080_enableDRDYInterrupt(void)
{
	uint8_t regContents;
	regContents = HDC2080_readReg(INTERRUPT_CONFIG);

	regContents = (regContents | 0x80);

	HDC2080_writeReg(INTERRUPT_CONFIG, regContents);
}

// disables the interrupt pin for DRDY operation
void HDC2080_disableDRDYInterrupt(void)
{
	uint8_t regContents;
	regContents = HDC2080_readReg(INTERRUPT_CONFIG);

	regContents = (regContents & 0x7F);

	HDC2080_writeReg(INTERRUPT_CONFIG, regContents);
}

float temperature = 0, humidity = 0;

void Init_HDC2080(void) {

	// Begin with a device reset
	HDC2080_reset();
	HDC2080_disableInterrupt();

	// Set up the comfort zone
	HDC2080_setHighTemp(100);         // High temperature of 28C
	HDC2080_setLowTemp(-40);          // Low temperature of 22C
	HDC2080_setHighHumidity(99);     // High humidity of 55%
	HDC2080_setLowHumidity(1);      // Low humidity of 40%

	// Configure Measurements
	HDC2080_setMeasurementMode(TEMP_AND_HUMID);  // Set measurements to temperature and humidity
	HDC2080_setRate(ONE_HZ);                     // Set measurement frequency to 1 Hz
	HDC2080_setTempRes(FOURTEEN_BIT);
	HDC2080_setHumidRes(FOURTEEN_BIT);

	//begin measuring
	HDC2080_triggerMeasurement();

}

void Read_HDC2080(float *temp, float *hum) {

  *temp = HDC2080_readTemp();
  *hum = HDC2080_readHumidity();

}
