/**
 ******************************************************************************
 * @file    LIS3DHHSensor.h
 * @author  CLab
 * @version V1.0.0
 * @date    25 July 2019
 * @brief   Abstract Class of an LIS3DHH Inertial Measurement Unit (IMU) 3 axes
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


/* Prevent recursive inclusion -----------------------------------------------*/

#ifndef __LIS3DHHSensor_H__
#define __LIS3DHHSensor_H__


/* Includes ------------------------------------------------------------------*/

#include "SPI.h"
#include "lis3dhh_reg.h"

/* Defines -------------------------------------------------------------------*/

#define LIS3DHH_ACC_SENSITIVITY   0.076f  /**< Sensitivity value for 2.5g full scale [mg/LSB] */

/* Typedefs ------------------------------------------------------------------*/
typedef enum
{
  LIS3DHH_STATUS_OK = 0,
  LIS3DHH_STATUS_ERROR
} LIS3DHHStatusTypeDef;

typedef enum
{
  LIS3DHH_INT1_PIN,
  LIS3DHH_INT2_PIN,
} LIS3DHH_SensorIntPin_t;

/* Class Declaration ---------------------------------------------------------*/
   
/**
 * Abstract class of an LIS3DHH Inertial Measurement Unit (IMU) 3 axes
 * sensor.
 */
class LIS3DHHSensor
{
  public:
    LIS3DHHSensor(SPIClass *spi, int cs_pin, uint32_t spi_speed=2000000);
    LIS3DHHStatusTypeDef begin(void);
    LIS3DHHStatusTypeDef end(void);
    LIS3DHHStatusTypeDef Enable_X(void);
    LIS3DHHStatusTypeDef Disable_X(void);
    LIS3DHHStatusTypeDef ReadID(uint8_t *id);
    LIS3DHHStatusTypeDef Get_X_Axes(int32_t *acceleration);
    LIS3DHHStatusTypeDef Get_X_Sensitivity(float *sensitivity);
    LIS3DHHStatusTypeDef Get_X_AxesRaw(int16_t *value);
    LIS3DHHStatusTypeDef Get_X_ODR(float *odr);
    LIS3DHHStatusTypeDef Set_X_ODR(float odr);
    LIS3DHHStatusTypeDef Get_X_FS(float *full_scale);
    LIS3DHHStatusTypeDef Set_X_FS(float full_scale);
    LIS3DHHStatusTypeDef Enable_DRDY_Interrupt(LIS3DHH_SensorIntPin_t int_pin);
    LIS3DHHStatusTypeDef Disable_DRDY_Interrupt(void);
    LIS3DHHStatusTypeDef Set_Filter_Mode(uint8_t filter_mode);
    LIS3DHHStatusTypeDef Get_DRDY_Status(uint8_t *status);
    LIS3DHHStatusTypeDef Get_FIFO_Num_Samples(uint16_t *num_samples);
    LIS3DHHStatusTypeDef Set_FIFO_Mode(uint8_t mode);
    LIS3DHHStatusTypeDef ReadReg(uint8_t reg, uint8_t *data);
    LIS3DHHStatusTypeDef WriteReg(uint8_t reg, uint8_t data);
    
    /**
     * @brief Utility function to read data.
     * @param  pBuffer: pointer to data to be read.
     * @param  RegisterAddr: specifies internal address register to be read.
     * @param  NumByteToRead: number of bytes to be read.
     * @retval 0 if ok, an error code otherwise.
     */
    uint8_t IO_Read(uint8_t* pBuffer, uint8_t RegisterAddr, uint16_t NumByteToRead)
    {        
      dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE3));

      digitalWrite(cs_pin, LOW);

      /* Write Reg Address */
      dev_spi->transfer(RegisterAddr | 0x80);
      /* Read the data */
      for (uint16_t i=0; i<NumByteToRead; i++) {
        *(pBuffer+i) = dev_spi->transfer(0x00);
      }
         
      digitalWrite(cs_pin, HIGH);

      dev_spi->endTransaction();

      return 0;
    }
    
    /**
     * @brief Utility function to write data.
     * @param  pBuffer: pointer to data to be written.
     * @param  RegisterAddr: specifies internal address register to be written.
     * @param  NumByteToWrite: number of bytes to write.
     * @retval 0 if ok, an error code otherwise.
     */
    uint8_t IO_Write(uint8_t* pBuffer, uint8_t RegisterAddr, uint16_t NumByteToWrite)
    {  
      dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE3));

      digitalWrite(cs_pin, LOW);

      /* Write Reg Address */
      dev_spi->transfer(RegisterAddr);
      /* Write the data */
      for (uint16_t i=0; i<NumByteToWrite; i++) {
        dev_spi->transfer(pBuffer[i]);
      }

      digitalWrite(cs_pin, HIGH);

      dev_spi->endTransaction();

      return 0;
    }

  private:

    /* Helper classes. */
    SPIClass *dev_spi;
    
    /* Configuration */
    int cs_pin;
    uint32_t spi_speed;
    
    uint8_t X_isEnabled;

    lis3dhh_ctx_t reg_ctx;
};

#ifdef __cplusplus
 extern "C" {
#endif
int32_t LIS3DHH_io_write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite );
int32_t LIS3DHH_io_read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead );
#ifdef __cplusplus
  }
#endif

#endif
