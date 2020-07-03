
//#define STEVAL_MKI109V3
#define NUCLEO_F411RE_X_NUCLEO_IKS01A2

#if defined(STEVAL_MKI109V3)
/* MKI109V3: Define communication interface */
#define SENSOR_BUS hspi2

/* MKI109V3: Vdd and Vddio power supply values */
#define PWM_3V3 915

#elif defined(NUCLEO_F411RE_X_NUCLEO_IKS01A2)
/* NUCLEO_F411RE_X_NUCLEO_IKS01A2: Define communication interface */
#define SENSOR_BUS hi2c1

#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "macro.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "gpio.h"

#include <string.h>
#include <stdio.h>
#include "iis2mdc_reg.h"


typedef union{
  int16_t i16bit[3];
  uint8_t u8bit[6];
} axis3bit16_t;

typedef union{
  int16_t i16bit;
  uint8_t u8bit[2];
} axis1bit16_t;

/* Private macro -------------------------------------------------------------*/

#define READ_TEMP		(0)

/* Private variables ---------------------------------------------------------*/
static axis3bit16_t data_raw_magnetic;
static axis1bit16_t data_raw_temperature;
static float temperature_degC;
static uint8_t whoamI, rst;
static stmdev_ctx_t dev_ctx;

/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);


/* Main Example --------------------------------------------------------------*/
uint8_t Init_IIS2MDC(void)
{
  /*
   *  Initialize mems driver interface
   */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &hi2c1;

  /*
   *  Check device ID
   */
  whoamI = 0;
  iis2mdc_device_id_get(&dev_ctx, &whoamI);
  if ( whoamI != IIS2MDC_ID )
    return 1; /*manage here device not found */
  /*
   *  Restore default configuration
   */
  iis2mdc_reset_set(&dev_ctx, PROPERTY_ENABLE);
  do {
    iis2mdc_reset_get(&dev_ctx, &rst);
  } while (rst);
  /*
   *  Enable Block Data Update
   */
  iis2mdc_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /*
   * Set Output Data Rate
   */
  iis2mdc_data_rate_set(&dev_ctx, IIS2MDC_ODR_10Hz);
  /*
   * Set / Reset sensor mode
   */
  iis2mdc_set_rst_mode_set(&dev_ctx, IIS2MDC_SENS_OFF_CANC_EVERY_ODR);
  /*
   * Enable temperature compensation
   */
  iis2mdc_offset_temp_comp_set(&dev_ctx, PROPERTY_ENABLE);
  /*
   * Enable INT on DATA READY
   */
  iis2mdc_drdy_on_pin_set(&dev_ctx, PROPERTY_ENABLE);
  /*
   * Set device in continuos mode
   */
  iis2mdc_operating_mode_set(&dev_ctx, IIS2MDC_CONTINUOUS_MODE);
  //iis2mdc_operating_mode_set(&dev_ctx, IIS2MDC_POWER_DOWN);

  return 0;

}

void Read_IIS2MDC(float *magnetic){

	/*
	 * Read output only if new value is available
	 */
	iis2mdc_reg_t reg;
	iis2mdc_status_get(&dev_ctx, &reg.status_reg);

	if (reg.status_reg.zyxda)
	{
		/* Read magnetic field data */
		memset(data_raw_magnetic.u8bit, 0x00, 3*sizeof(int16_t));
		iis2mdc_magnetic_raw_get(&dev_ctx, data_raw_magnetic.u8bit);
		magnetic[0] = IIS2MDC_FROM_LSB_TO_mG( data_raw_magnetic.i16bit[0]);
		magnetic[1] = IIS2MDC_FROM_LSB_TO_mG( data_raw_magnetic.i16bit[1]);
		magnetic[2] = IIS2MDC_FROM_LSB_TO_mG( data_raw_magnetic.i16bit[2]);

#if READ_TEMP

		//sprintf((char*)tx_buffer, "Magnetic field [mG]:%4.2f\t%4.2f\t%4.2f\r\n",
		//       magnetic_mG[0], magnetic_mG[1], magnetic_mG[2]);
		//tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );

		/* Read temperature data */
		memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
		iis2mdc_temperature_raw_get(&dev_ctx, data_raw_temperature.u8bit);
		temperature_degC = IIS2MDC_FROM_LSB_TO_degC( data_raw_temperature.i16bit );

		//sprintf((char*)tx_buffer, "Temperature [degC]:%6.2f\r\n", temperature_degC );
		//tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
#endif

	}

}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len)
{
  if (handle == &hi2c1)
  {
    HAL_I2C_Mem_Write(handle, IIS2MDC_I2C_ADD, reg,
                      I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  }
  return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
  if (handle == &hi2c1)
  {
    HAL_I2C_Mem_Read(handle, IIS2MDC_I2C_ADD, reg,
                     I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  }
  return 0;
}

