/**
* Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
*
* BSD-3-Clause
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* @file       bme280.h
* @date       2020-03-28
* @version    v3.5.0
*
*/

/*! @file bme280.h
 * @brief Sensor driver for BME280 sensor
 */

/*!
 * @defgroup bme280 BME280
 * @brief <a href="https://www.bosch-sensortec.com/bst/products/all_products/bme280">Product Overview</a>
 * and  <a href="https://github.com/BoschSensortec/BME280_driver">Sensor API Source Code</a>
 */

#ifndef MAX30102_H_
#define MAX30102_H_

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

/* Header includes */
#include "esp_log.h"
#include "max30102_defs.h"
/*! @todo Implement build environment variable to select driver header */
#include "driver/MAX30102_ESP32C3.h"

/**
 * \ingroup max30102
 * \defgroup max30102ApiInit Initialization
 * @brief Initialize the sensor and device structure
 */

/*!
 * \ingroup max30102ApiInit
 * \page max30102_api_max30102_init max30102_init
 * \code
 * int8_t max30102_init(struct max30102_dev *dev);
 * \endcode
 * @details This API reads the chip-id of the sensor to verify it
 * As this API is the entry point, call this API before using other APIs.
 *
 * @param[in,out] dev : Structure instance of max30102_dev
 *
 * @return Result of API execution status.
 *
 * @retval   0 -> Success.
 * @retval > 0 -> Warning.
 * @retval < 0 -> Fail.
 *
 */
int8_t max30102_init(struct max30102_dev *dev);

/**
 * \ingroup max30102
 * \defgroup max30102ApiRegister Registers
 * @brief Generic API for accessing sensor registers
 */

/*!
 * \ingroup bme280ApiRegister
 * \page bme280_api_bme280_set_regs bme280_set_regs
 * \code
 * int8_t bme280_set_regs(const uint8_t reg_addr, const uint8_t *reg_data, uint8_t len, struct bme280_dev *dev);
 * \endcode
 * @details This API writes the given data to the register address of the sensor
 *
 * @param[in] reg_addr : Register addresses to where the data is to be written
 * @param[in] reg_data : Pointer to data buffer which is to be written
 *                       in the reg_addr of sensor.
 * @param[in] len      : No of bytes of data to write
 * @param[in,out] dev  : Structure instance of bme280_dev
 *
 * @return Result of API execution status.
 *
 * @retval   0 -> Success.
 * @retval > 0 -> Warning.
 * @retval < 0 -> Fail.
 *
 */
int8_t max30102_set_regs(uint8_t *reg_addr, const uint8_t *reg_data, uint8_t len, struct max30102_dev *dev);

/*!
 * \ingroup bme280ApiRegister
 * \page bme280_api_bme280_get_regs bme280_get_regs
 * \code
 * int8_t bme280_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint8_t len, struct bme280_dev *dev);
 * \endcode
 * @details This API reads the data from the given register address of sensor.
 *
 * @param[in] reg_addr  : Register address from where the data to be read
 * @param[out] reg_data : Pointer to data buffer to store the read data.
 * @param[in] len       : No of bytes of data to be read.
 * @param[in,out] dev   : Structure instance of bme280_dev.
 *
 * @return Result of API execution status.
 *
 * @retval   0 -> Success.
 * @retval > 0 -> Warning.
 * @retval < 0 -> Fail.
 *
 */
int8_t max30102_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, struct max30102_dev *dev);

/**
 * \ingroup bme280
 * \defgroup bme280ApiSensorSettings Sensor Settings
 * @brief Generic API for accessing sensor settings
 */
/*!
 * @details This API sets FIFO parameters
 * @param[in] dev : Structure instance of MAX30102.
 * @param[in] setup : Variable which contains setup for the FIFO.
 * 
 * @return Result of API execution status.
 */
int8_t max30102_set_fifo(uint8_t setup, struct max30102_dev *dev);

/*!
 * @details This API sets LED current amplitude
 * @param[in] dev : Structure instance of MAX30102.
 * @param[in] amplitude : Variable which contains discrete value mapped to current value (see datasheet).
 * 
 * @return Result of API execution status.
 */
int8_t max30102_set_led_amplitude(uint8_t amplitude, struct max30102_dev *dev);

/*!
 * @details This API sets sensor mode
 * @param[in] dev : Structure instance of MAX30102.
 * @param[in] sensor_mode : Variable which contains setup for sensor adquisition mode.
 * 
 * @return Result of API execution status.
 */
int8_t max30102_set_spo2(uint8_t sensor_mode, struct max30102_dev *dev);

/*!
 * \ingroup max30102ApiSensorMode
 * \page max30102_api_max30102_set_sensor_mode max30102_set_sensor_mode
 * \code
 * int8_t max30102_set_sensor_mode(uint8_t sensor_mode, const struct max30102_dev *dev);
 * \endcode
 * @details This API sets the adquisition mode of the sensor.
 *
 * @param[in] dev : Structure instance of max30102_dev.
 * @param[in] sensor_mode : Variable which contains the mode to be set.
 *
 * - MAX30102_SHDWN: Shutdown sensor
 * - MAX30102_RESET: Reset sensor
 * - MAX30102_HR_MODE: Heart rate mode
 * - MAX30102_SPO2_MODE: SpO2 mode
 * - MAX30102_MULTILED_MODE: Heart rate and SpO2 mode.
 *
 * @return Result of API execution status
 *
 * @retval   0 -> Success.
 * @retval > 0 -> Warning.
 * @retval < 0 -> Fail.
 *
 */
int8_t max30102_set_sensor_mode(uint8_t sensor_mode, struct max30102_dev *dev);

/**
 * \ingroup max30102
 * \defgroup max30102ApiSensorData Sensor Data
 * @brief Data processing of sensor
 */
/*!
 * \ingroup max30102ApiSensorData
 * \page max30102_api_max30102_get_sensor_data max30102_get_sensor_data
 * \code
 * int8_t max30102_get_sensor_data(uint8_t sensor_comp, struct max30102_data *comp_data, struct max30102_dev *dev);
 * \endcode
 * @details This API reads the sample stored in the FIFO
 *
 * @param[in] sensor_comp : Variable which selects which data to be read from
 * the sensor.
 *
 * MAX30102_BPM
 * MAX30102_ALL
 *
 * @param[out] comp_data : Structure instance of max30102_data.
 * @param[in] dev : Structure instance of max30102_dev.
 *
 * @return Result of API execution status
 *
 * @retval   0 -> Success.
 * @retval > 0 -> Warning.
 * @retval < 0 -> Fail.
 *
 * @todo Implement read for SpO2 and multiled mode.
 */
int8_t max30102_get_sensor_data(uint8_t sensor_comp, struct max30102_data *comp_data, struct max30102_dev *dev);

/*!
 * \ingroup max30102ApiSensorData
 * \page max30102_api_max30102_parse_sensor_data max30102_parse_sensor_data
 * \code
 * void max30102_parse_sensor_data(const uint8_t *reg_data, struct max30102_data *data);
 * \endcode
 *  @details This API is used to parse the sample read (see the datasheet for
 * more information about the samples format vs. ADC resolution)
 *
 *  @param[in] reg_data     : Contains register data which needs to be parsed
 *  @param[out] uncomp_data : Contains the uncompensated pressure, temperature
 *  and humidity data.
 *
 * @todo Implement parse for several ADC ranges.
 */
void max30102_parse_sensor_data(const uint8_t *reg_data, struct max30102_data *data);

/*!
 * \ingroup max30102ApiSensorData
 * \page max30102_api_max30102_get_bpm max30102_get_bpm
 * \code
 * void max30102_get_bpm(int32_t *data);
 * \endcode
 *  @details This API is used to process the buffer data in order to get the
 * BPM value through edge detection.
 *
 *  @param[in] data : The buffer with samples to process
 *
 */
uint8_t max30102_get_bpm(int32_t *data);

#ifdef __cplusplus
}
#endif /* End of CPP guard */
#endif /* BME280_H_ */
/** @}*/
