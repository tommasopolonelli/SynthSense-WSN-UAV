/*
 ******************************************************************************
 * @file    read_data_interrupt.c
 * @author  Sensors Software Solution Team
 * @brief   This file show the simplest way to get data from sensor (interrupt
 * 			mode).
 *
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

/*
 * This example was developed using the following STMicroelectronics
 * evaluation boards:
 *
 * - STEVAL_MKI109V3
 * - NUCLEO_F411RE + X_NUCLEO_IKS01A2
 *
 * and STM32CubeMX tool with STM32CubeF4 MCU Package
 *
 * Used interfaces:
 *
 * STEVAL_MKI109V3    - Host side:   USB (Virtual COM)
 *                    - Sensor side: SPI(Default) / I2C(supported)
 *
 * NUCLEO_STM32F411RE + X_NUCLEO_IKS01A2 - Host side: UART(COM) to USB bridge
 *                                       - I2C(Default) / SPI(N/A)
 *
 * If you need to run this example on a different hardware platform a
 * modification of the functions: `platform_write`, `platform_read`,
 * `tx_com` and 'platform_init' is required.
 *
 */

/* STMicroelectronics evaluation boards definition
 *
 * Please uncomment ONLY the evaluation boards in use.
 * If a different hardware is used please comment all
 * following target board and redefine yours.
 */
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
#include <string.h>
#include <stdio.h>

#include "main.h"
#include "macro.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "gpio.h"

#include "lsm6dsox_reg.h"


typedef union{
  int16_t i16bit[3];
  uint8_t u8bit[6];
} axis3bit16_t;

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static axis3bit16_t data_raw_acceleration;
static axis3bit16_t data_raw_angular_rate;
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
static void platform_init(void);

/* Main Example --------------------------------------------------------------*/
uint8_t Init_lsm6dsox(void)
{

  /*
   * Uncomment to configure INT 1
   */
  lsm6dsox_pin_int1_route_t int1_route;

  /*
   * Uncomment to configure INT 2
   */
  lsm6dsox_pin_int2_route_t int2_route;

  /*
   *  Initialize mems driver interface
   */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &hi2c1;

  /*
   * Init test platform
   */
  platform_init();

  /*
   *  Check device ID
   */
  lsm6dsox_device_id_get(&dev_ctx, &whoamI);
  if (whoamI != LSM6DSOX_ID)
    return 1;

  /*
   *  Restore default configuration
   */
  lsm6dsox_reset_set(&dev_ctx, PROPERTY_ENABLE);
  do {
    lsm6dsox_reset_get(&dev_ctx, &rst);
  } while (rst);

  /*
   * Disable I3C interface
   */
  lsm6dsox_i3c_disable_set(&dev_ctx, LSM6DSOX_I3C_DISABLE);

  /*
   *  Enable Block Data Update
   */
  lsm6dsox_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

  /*
   * Set Output Data Rate
   */
  lsm6dsox_xl_data_rate_set(&dev_ctx, LSM6DSOX_XL_ODR_12Hz5);
  //lsm6dsox_xl_data_rate_set(&dev_ctx, LSM6DSOX_XL_ODR_OFF);
  lsm6dsox_gy_data_rate_set(&dev_ctx, LSM6DSOX_GY_ODR_12Hz5);
  //lsm6dsox_gy_data_rate_set(&dev_ctx, LSM6DSOX_GY_ODR_OFF);

  /*
   * Set full scale
   */
  lsm6dsox_xl_full_scale_set(&dev_ctx, LSM6DSOX_2g);
  lsm6dsox_gy_full_scale_set(&dev_ctx, LSM6DSOX_2000dps);

  /*
   * Enable drdy 75 μs pulse: uncomment if interrupt must be pulsed
   */
  //lsm6dsox_data_ready_mode_set(&dev_ctx, LSM6DSOX_DRDY_PULSED);

  /*
   * Uncomment if interrupt generation on Free Fall INT1 pin
   */
  lsm6dsox_pin_int1_route_get(&dev_ctx, &int1_route);
  int1_route.md1_cfg.int1_ff = PROPERTY_ENABLE;
  lsm6dsox_pin_int1_route_set(&dev_ctx, &int1_route);

  /*
   * Uncomment if interrupt generation on Free Fall INT2 pin
   */
  lsm6dsox_pin_int2_route_get(&dev_ctx, &int2_route);
  int2_route.md2_cfg.int2_ff = PROPERTY_ENABLE;
  lsm6dsox_pin_int2_route_set(&dev_ctx, &int2_route);

  return 0;

}


void Read_lsm6dsox (float *acc, float *angular){


    uint8_t reg;

    /*
     * Read output only if new xl value is available
     */
    lsm6dsox_xl_flag_data_ready_get(&dev_ctx, &reg);
    if (reg)
    {
      /*
       * Read acceleration field data
       */
      memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
      lsm6dsox_acceleration_raw_get(&dev_ctx, data_raw_acceleration.u8bit);
      acc[0] =
        lsm6dsox_from_fs2_to_mg(data_raw_acceleration.i16bit[0]);
      acc[1] =
        lsm6dsox_from_fs2_to_mg(data_raw_acceleration.i16bit[1]);
      acc[2] =
        lsm6dsox_from_fs2_to_mg(data_raw_acceleration.i16bit[2]);

      /*sprintf((char*)tx_buffer, "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
              acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
      tx_com(tx_buffer, strlen((char const*)tx_buffer));*/
    }

    /*
     * Read output only if new gyro value is available
     */
    lsm6dsox_gy_flag_data_ready_get(&dev_ctx, &reg);
    if (reg)
    {
      /*
       * Read angular rate field data
       */
      memset(data_raw_angular_rate.u8bit, 0x00, 3 * sizeof(int16_t));
      lsm6dsox_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate.u8bit);
      angular[0] =
        lsm6dsox_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[0]);
      angular[1] =
        lsm6dsox_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[1]);
      angular[2] =
        lsm6dsox_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[2]);

      /*sprintf((char*)tx_buffer, "Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
              angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
      tx_com(tx_buffer, strlen((char const*)tx_buffer));*/
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
    HAL_I2C_Mem_Write(handle, LSM6DSOX_I2C_ADD_H, reg,
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
    HAL_I2C_Mem_Read(handle, LSM6DSOX_I2C_ADD_H, reg,
                     I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  }
  return 0;
}



/*
 * @brief  platform specific initialization (platform dependent)
 */
static void platform_init(void)
{

}
