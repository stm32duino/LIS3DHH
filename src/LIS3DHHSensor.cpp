/**
 ******************************************************************************
 * @file    LIS3DHHSensor.cpp
 * @author  CLab
 * @version V1.0.0
 * @date    25 July 2019
 * @brief   Implementation of an LIS3DHH Inertial Measurement Unit (IMU) 3 axes
 *          sensor.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2019 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */


/* Includes ------------------------------------------------------------------*/

#include "LIS3DHHSensor.h"


/* Class Implementation ------------------------------------------------------*/

/** Constructor
 * @param spi object of an helper class which handles the SPI peripheral
 * @param cs_pin the chip select pin
 * @param spi_speed the SPI speed
 */
LIS3DHHSensor::LIS3DHHSensor(SPIClass *spi, int cs_pin, uint32_t spi_speed) : dev_spi(spi), cs_pin(cs_pin), spi_speed(spi_speed)
{
  reg_ctx.write_reg = LIS3DHH_io_write;
  reg_ctx.read_reg = LIS3DHH_io_read;
  reg_ctx.handle = (void *)this;
  X_isEnabled = 0U;
}

/**
 * @brief  Configure the sensor in order to be used
 * @retval 0 in case of success, an error code otherwise
 */
LIS3DHHStatusTypeDef LIS3DHHSensor::begin()
{
  // Configure CS pin
  pinMode(cs_pin, OUTPUT);
  digitalWrite(cs_pin, HIGH); 

  /* Enable register address automatically incremented during a multiple byte
  access with a serial interface. */
  if (lis3dhh_auto_add_inc_set(&reg_ctx, PROPERTY_ENABLE) != 0)
  {
    return LIS3DHH_STATUS_ERROR;
  }

  /* Enable BDU */
  if (lis3dhh_block_data_update_set(&reg_ctx, PROPERTY_ENABLE) != 0)
  {
    return LIS3DHH_STATUS_ERROR;
  }

  /* FIFO mode selection */
  if (lis3dhh_fifo_mode_set(&reg_ctx, LIS3DHH_BYPASS_MODE) != 0)
  {
    return LIS3DHH_STATUS_ERROR;
  }

  /* Output data rate selection - power down. */
  if (lis3dhh_data_rate_set(&reg_ctx, LIS3DHH_POWER_DOWN) != 0)
  {
    return LIS3DHH_STATUS_ERROR;
  }

  X_isEnabled = 0;

  return LIS3DHH_STATUS_OK;
}

/**
 * @brief  Disable the sensor and relative resources
 * @retval 0 in case of success, an error code otherwise
 */
LIS3DHHStatusTypeDef LIS3DHHSensor::end()
{
  /* Disable acc */
  if (Disable_X() != LIS3DHH_STATUS_OK)
  {
    return LIS3DHH_STATUS_ERROR;
  }

  /* Reset CS configuration */
  pinMode(cs_pin, INPUT); 

  return LIS3DHH_STATUS_OK;
}

/**
 * @brief  Enable LIS3DHH Accelerator
 * @retval 0 in case of success, an error code otherwise
 */
LIS3DHHStatusTypeDef LIS3DHHSensor::Enable_X(void)
{ 
  /* Check if the component is already enabled */
  if (X_isEnabled == 1U)
  {
    return LIS3DHH_STATUS_OK;
  }
  
  /* Output data rate selection - power down. */
  if (lis3dhh_data_rate_set(&reg_ctx, LIS3DHH_1kHz1) != 0)
  {
    return LIS3DHH_STATUS_ERROR;
  }
  
  X_isEnabled = 1U;
  
  return LIS3DHH_STATUS_OK;
}

/**
 * @brief  Disable LIS3DHH Accelerator
 * @retval 0 in case of success, an error code otherwise
 */
LIS3DHHStatusTypeDef LIS3DHHSensor::Disable_X(void)
{ 
  /* Check if the component is already disabled */
  if (X_isEnabled == 0U)
  {
    return LIS3DHH_STATUS_OK;
  }

  /* Output data rate selection - power down. */
  if (lis3dhh_data_rate_set(&reg_ctx, LIS3DHH_POWER_DOWN) != 0)
  {
    return LIS3DHH_STATUS_ERROR;
  }
  
  X_isEnabled = 0U;
  
  return LIS3DHH_STATUS_OK;
}

/**
 * @brief  Read ID of LIS3DHH Accelerometer and Gyroscope
 * @param  id the pointer where the ID of the device is stored
 * @retval 0 in case of success, an error code otherwise
 */
LIS3DHHStatusTypeDef LIS3DHHSensor::ReadID(uint8_t *id)
{
  if(!id)
  { 
    return LIS3DHH_STATUS_ERROR;
  }

  /* Read WHO AM I register */
  if (lis3dhh_device_id_get(&reg_ctx, id) != 0)
  {
    return LIS3DHH_STATUS_ERROR;
  }

  return LIS3DHH_STATUS_OK;
}

/**
 * @brief  Read data from LIS3DHH Accelerometer
 * @param  acceleration the pointer where the accelerometer data are stored
 * @retval 0 in case of success, an error code otherwise
 */
LIS3DHHStatusTypeDef LIS3DHHSensor::Get_X_Axes(int32_t *acceleration)
{
  int16_t data_raw[3];
  float sensitivity = 0;
  
  /* Read raw data from LIS3DHH output register. */
  if (Get_X_AxesRaw(data_raw) != LIS3DHH_STATUS_OK)
  {
    return LIS3DHH_STATUS_ERROR;
  }
  
  /* Get LIS3DHH actual sensitivity. */
  if (Get_X_Sensitivity(&sensitivity) != LIS3DHH_STATUS_OK)
  {
    return LIS3DHH_STATUS_ERROR;
  }
  
  /* Calculate the data. */
  acceleration[0] = (int32_t)(data_raw[0] * sensitivity);
  acceleration[1] = (int32_t)(data_raw[1] * sensitivity);
  acceleration[2] = (int32_t)(data_raw[2] * sensitivity);
  
  return LIS3DHH_STATUS_OK;
}

/**
 * @brief  Read Accelerometer Sensitivity
 * @param  sensitivity the pointer where the accelerometer sensitivity is stored
 * @retval 0 in case of success, an error code otherwise
 */
LIS3DHHStatusTypeDef LIS3DHHSensor::Get_X_Sensitivity(float *sensitivity)
{
  /* There is only one value of sensitivity */
  *sensitivity = LIS3DHH_ACC_SENSITIVITY;

  return LIS3DHH_STATUS_OK;
}

/**
 * @brief  Read raw data from LIS3DHH Accelerometer
 * @param  value the pointer where the accelerometer raw data are stored
 * @retval 0 in case of success, an error code otherwise
 */
LIS3DHHStatusTypeDef LIS3DHHSensor::Get_X_AxesRaw(int16_t *value)
{
  axis3bit16_t data_raw;
  LIS3DHHStatusTypeDef ret = LIS3DHH_STATUS_OK;

  /* Read raw data values. */
  if (lis3dhh_acceleration_raw_get(&reg_ctx, data_raw.u8bit) != 0)
  {
    return LIS3DHH_STATUS_ERROR;
  }

  value[0] = (data_raw.i16bit[0]);
  value[1] = (data_raw.i16bit[1]);
  value[2] = (data_raw.i16bit[2]);

  return ret;
}

/**
 * @brief  Read LIS3DHH Accelerometer output data rate
 * @param  odr the pointer to the output data rate
 * @retval 0 in case of success, an error code otherwise
 */
LIS3DHHStatusTypeDef LIS3DHHSensor::Get_X_ODR(float* odr)
{
  /* There is only one value of ODR */
  *odr = 1100.0f;

  return LIS3DHH_STATUS_OK;
}

/**
 * @brief  Set LIS3DHH Accelerometer output data rate
 * @param  odr the output data rate to be set
 * @retval 0 in case of success, an error code otherwise
 */
LIS3DHHStatusTypeDef LIS3DHHSensor::Set_X_ODR(float odr)
{
  (void)(odr);
  return LIS3DHH_STATUS_OK;
}

/**
 * @brief  Read LIS3DHH Accelerometer full scale
 * @param  full_scale the pointer to the full scale
 * @retval 0 in case of success, an error code otherwise
 */
LIS3DHHStatusTypeDef LIS3DHHSensor::Get_X_FS(float* full_scale)
{
  /* There is only one value of FS */
  *full_scale = 2.5f;

  return LIS3DHH_STATUS_OK;
}

/**
 * @brief  Set LIS3DHH Accelerometer full scale
 * @param  full_scale the full scale to be set
 * @retval 0 in case of success, an error code otherwise
 */
LIS3DHHStatusTypeDef LIS3DHHSensor::Set_X_FS(float full_scale)
{
  (void)(full_scale);
  return LIS3DHH_STATUS_OK;
}

/**
 * @brief  Enable Data Ready Interrupt
 * @param  int_pin interrupt pin line to be used
 * @retval 0 in case of success, an error code otherwise
 */
LIS3DHHStatusTypeDef LIS3DHHSensor::Enable_DRDY_Interrupt(LIS3DHH_SensorIntPin_t int_pin)
{
  LIS3DHHStatusTypeDef ret = LIS3DHH_STATUS_OK;

  /* Enable Data Ready Interrupt on either INT1 or INT2 pin */
  switch (int_pin)
  {
    case LIS3DHH_INT1_PIN:
      /* Interrupt set to INT1 */
      if(lis3dhh_drdy_on_int1_set(&reg_ctx, PROPERTY_ENABLE) != 0)
      {
        return LIS3DHH_STATUS_ERROR;
      }
      break;
    case LIS3DHH_INT2_PIN:
      /* Interrupt set to INT2 */
      if(lis3dhh_drdy_on_int2_set(&reg_ctx, PROPERTY_ENABLE) != 0)
      {
        return LIS3DHH_STATUS_ERROR;
      }
      break;
    default:
      ret = LIS3DHH_STATUS_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Disable Data Ready Interrupt
 * @retval 0 in case of success, an error code otherwise
 */
LIS3DHHStatusTypeDef LIS3DHHSensor::Disable_DRDY_Interrupt(void)
{
  if(lis3dhh_drdy_on_int1_set(&reg_ctx, PROPERTY_DISABLE) != 0)
  {
    return LIS3DHH_STATUS_ERROR;
  }

  if(lis3dhh_drdy_on_int2_set(&reg_ctx, PROPERTY_DISABLE) != 0)
  {
    return LIS3DHH_STATUS_ERROR;
  }

  return LIS3DHH_STATUS_OK;
}

/**
 * @brief  Set Filter Mode
 * @param  filter_mode filter mode to be set
 * @retval 0 in case of success, an error code otherwise
 */
LIS3DHHStatusTypeDef LIS3DHHSensor::Set_Filter_Mode(uint8_t filter_mode)
{
  if(lis3dhh_filter_config_set(&reg_ctx, (lis3dhh_dsp_t)filter_mode) != 0)
  {
    return LIS3DHH_STATUS_ERROR;
  }
  
  return LIS3DHH_STATUS_OK;
}

/**
 * @brief  Get Data Ready status
 * @param  status the Data Ready status
 * @retval 0 in case of success, an error code otherwise
 */
LIS3DHHStatusTypeDef LIS3DHHSensor::Get_DRDY_Status(uint8_t *status)
{
  if(lis3dhh_xl_data_ready_get(&reg_ctx, status) != 0)
  {
    return LIS3DHH_STATUS_ERROR;
  }

  return LIS3DHH_STATUS_OK;
}


/**
 * @brief  Get the number of samples contained into the FIFO
 * @param  num_samples the number of samples contained into the FIFO
 * @retval 0 in case of success, an error code otherwise
 */
LIS3DHHStatusTypeDef LIS3DHHSensor::Get_FIFO_Num_Samples(uint16_t *num_samples)
{
  lis3dhh_fifo_src_t fifo_samples;

  if (lis3dhh_read_reg(&reg_ctx, LIS3DHH_FIFO_SRC, (uint8_t *)&fifo_samples, 1) != 0)
  {
    return LIS3DHH_STATUS_ERROR;
  }

  if(fifo_samples.fss == 0x20)
  {
    *num_samples = 32;
  }
  else
  {
    *num_samples = fifo_samples.fss;
  }

  return LIS3DHH_STATUS_OK;
}

/**
 * @brief  Set the FIFO mode
 * @param  mode FIFO mode
 * @retval 0 in case of success, an error code otherwise
 */
LIS3DHHStatusTypeDef LIS3DHHSensor::Set_FIFO_Mode(uint8_t mode)
{
  LIS3DHHStatusTypeDef ret = LIS3DHH_STATUS_OK;

  /* Verify that the passed parameter contains one of the valid values. */
  switch ((lis3dhh_fmode_t)mode)
  {
    case LIS3DHH_BYPASS_MODE:
    case LIS3DHH_FIFO_MODE:
    case LIS3DHH_STREAM_TO_FIFO_MODE:
    case LIS3DHH_BYPASS_TO_STREAM_MODE:
    case LIS3DHH_DYNAMIC_STREAM_MODE:
      break;

    default:
      ret = LIS3DHH_STATUS_ERROR;
      break;
  }

  if (ret == LIS3DHH_STATUS_ERROR)
  {
    return ret;
  }

  if (lis3dhh_fifo_mode_set(&reg_ctx, (lis3dhh_fmode_t)mode) != 0)
  {
    ret = LIS3DHH_STATUS_ERROR;
  }

  return ret;
}

/**
 * @brief Read the data from register
 * @param reg register address
 * @param data register data
 * @retval 0 in case of success, an error code otherwise
 */
LIS3DHHStatusTypeDef LIS3DHHSensor::ReadReg(uint8_t reg, uint8_t *data)
{

  if (lis3dhh_read_reg(&reg_ctx, reg, data, 1) != 0)
  {
    return LIS3DHH_STATUS_ERROR;
  }

  return LIS3DHH_STATUS_OK;
}

/**
 * @brief Write the data to register
 * @param reg register address
 * @param data register data
 * @retval 0 in case of success, an error code otherwise
 */
LIS3DHHStatusTypeDef LIS3DHHSensor::WriteReg(uint8_t reg, uint8_t data)
{

  if (lis3dhh_write_reg(&reg_ctx, reg, &data, 1) != 0)
  {
    return LIS3DHH_STATUS_ERROR;
  }

  return LIS3DHH_STATUS_OK;
}


int32_t LIS3DHH_io_write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite)
{
  return ((LIS3DHHSensor *)handle)->IO_Write(pBuffer, WriteAddr, nBytesToWrite);
}

int32_t LIS3DHH_io_read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead)
{
  return ((LIS3DHHSensor *)handle)->IO_Read(pBuffer, ReadAddr, nBytesToRead);
}
