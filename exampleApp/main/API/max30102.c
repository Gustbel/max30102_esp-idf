/*! @file max30102.c
 * @brief API for MAX30102 Heart rate and SpO2 sensor
 */
#include "max30102.h"

/*!
 * @brief This internal API is used to validate the device pointer for
 * null conditions.
 *
 * @param[in] dev : Structure instance of bme280_dev.
 *
 * @return Result of API execution status
 *
 * @retval   0 -> Success.
 * @retval > 0 -> Warning.
 * @retval < 0 -> Fail.
 *
 */
static int8_t null_ptr_check(const struct max30102_dev *dev);

/*!
 * @brief This internal API interleaves the register address between the
 * register data buffer for burst write operation.
 *
 * @param[in] reg_addr   : Contains the register address array.
 * @param[out] temp_buff : Contains the temporary buffer to store the
 * register data and register address.
 * @param[in] reg_data   : Contains the register data to be written in the
 * temporary buffer.
 * @param[in] len        : No of bytes of data to be written for burst write.
 *
 */
static void interleave_reg_addr(const uint8_t *reg_addr, uint8_t *temp_buff, const uint8_t *reg_data, uint8_t len);

/****************** Global Function Definitions *******************************/

/*!
 *  @brief This API is the entry point.
 *  It reads the chip-id from the sensor.
 */
int8_t max30102_init(struct max30102_dev *dev)
{
    int8_t rslt;

    /* chip id read try count */
    uint8_t try_count = 5;
    uint8_t chip_id = 0;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == MAX30102_OK)
    {
        while (try_count)
        {
            /* Read the chip-id of bme280 sensor */
            rslt = max30102_get_regs(MAX30102_CHIP_ID_ADDR, &chip_id, 1, dev);

            /* Check for chip id validity */
            if ((rslt == MAX30102_OK) && (chip_id == MAX30102_CHIP_ID))
            {
                dev->chip_id = chip_id;
                break;
            }

            /* Wait for 1 ms */
            dev->delay_us(1000);
            --try_count;
        }

        /* Chip id check failed */
        if (!try_count)
        {
            rslt = MAX30102_E_DEV_NOT_FOUND;
        }
    }

    return rslt;
}

/*!
 * @brief This API writes the given data to the register address
 * of the sensor.
 */
int8_t max30102_set_regs(uint8_t *reg_addr, const uint8_t *reg_data, uint8_t len, struct max30102_dev *dev)
{
    int8_t rslt;
    uint8_t temp_buff[20]; /* Typically not to write more than 10 registers */

    if (len > 10)
    {
        len = 10;
    }

    uint16_t temp_len;
    uint8_t reg_addr_cnt;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Check for arguments validity */
    if ((rslt == MAX30102_OK) && (reg_addr != NULL) && (reg_data != NULL))
    {
        if (len != 0)
        {
            temp_buff[0] = reg_data[0];

            /* Burst write mode */
            if (len > 1)
            {
                /* Interleave register address w.r.t data for
                 * burst write
                 */
                interleave_reg_addr(reg_addr, temp_buff, reg_data, len);
                temp_len = ((len * 2) - 1);
            }
            else
            {
                temp_len = len;
            }

            dev->intf_rslt = dev->write(reg_addr[0], temp_buff, temp_len);

            /* Check for communication error */
            if (dev->intf_rslt != MAX30102_INTF_RET_SUCCESS)
            {
                rslt = MAX30102_E_COMM_FAIL;
            }
        }
        else
        {
            rslt = MAX30102_E_INVALID_LEN;
        }
    }
    else
    {
        rslt = MAX30102_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API reads the data from the given register address of the sensor.
 */
int8_t max30102_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, struct max30102_dev *dev)
{
    int8_t rslt;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if ((rslt == MAX30102_OK) && (reg_data != NULL))
    {
        /* Read the data  */
        dev->intf_rslt = dev->read(reg_addr, reg_data, len);

        /* Check for communication error */
        if (dev->intf_rslt != MAX30102_INTF_RET_SUCCESS)
        {
            rslt = MAX30102_E_COMM_FAIL;
        }
    }
    else
    {
        rslt = MAX30102_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API writes configuration data for FIFO
 */
int8_t max30102_set_fifo(uint8_t setup, struct max30102_dev *dev)
{
    int8_t rslt;
    uint8_t reg_addr = MAX30102_FIFO_CFG_ADDR;

    rslt = null_ptr_check(dev);

    if (rslt == MAX30102_OK)
    {
        rslt = max30102_set_regs(&reg_addr, &setup, 1, dev);
    }

    return rslt;
}

/*!
 * @brief This API writes current amplitude data for LEDs
 */
int8_t max30102_set_led_amplitude(uint8_t amplitude, struct max30102_dev *dev)
{
    int8_t rslt;
    uint8_t reg_addr = MAX30102_LED1_PA_ADDR;

    rslt = null_ptr_check(dev);

    if (rslt == MAX30102_OK)
    {
        rslt = max30102_set_regs(&reg_addr, &amplitude, 1, dev);
        reg_addr = MAX30102_LED2_PA_ADDR;

        dev->delay_us(10000);

        rslt = max30102_set_regs(&reg_addr, &amplitude, 1, dev);
    }

    return rslt;
}

/*!
 * @brief This API writes configuration to select the sensor mode
 */
int8_t max30102_set_spo2(uint8_t sensor_mode, struct max30102_dev *dev)
{
    int8_t rslt;
    uint8_t reg_addr = MAX30102_SPO2_CFG_ADDR;

    rslt = null_ptr_check(dev);

    if (rslt == MAX30102_OK)
    {
        rslt = max30102_set_regs(&reg_addr, &sensor_mode, 1, dev);
    }

    return rslt;
}

/*!
 * @brief This API sets the power mode of the sensor.
 */
int8_t max30102_set_sensor_mode(uint8_t sensor_mode, struct max30102_dev *dev)
{
    int8_t rslt;
    uint8_t reg_addr = MAX30102_MODE_CFG_ADDR;

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    if (rslt == MAX30102_OK)
    {
        rslt = max30102_set_regs(&reg_addr, &sensor_mode, 1, dev);
    }

    return rslt;
}

/*!
 * @brief This API reads the sample stored in the FIFO.
 */
int8_t max30102_get_sensor_data(uint8_t sensor_comp, struct max30102_data *comp_data, struct max30102_dev *dev)
{
    int8_t rslt;

    /* Array to store the samples retrieved */
    uint8_t reg_data[MAX30102_DATA_LEN] = { 0 };

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    if ((rslt == MAX30102_OK) && (comp_data != NULL))
    {
        rslt = max30102_get_regs(MAX30102_FIFO_DATA_ADDR, reg_data, MAX30102_DATA_LEN, dev);

        if (rslt == MAX30102_OK)
        {
            /* Parse the read data from the sensor */
            max30102_parse_sensor_data(reg_data, comp_data);
        }
    }
    else
    {
        rslt = MAX30102_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API is used to parse the samples.
 */
void max30102_parse_sensor_data(const uint8_t *reg_data, struct max30102_data *data)
{
    /* Variables to store the sensor data */
    uint32_t data_xlsb = 0;
    uint32_t data_lsb = 0;
    uint32_t data_msb = 0;

    data_msb = (uint32_t)reg_data[0] << 16;
    data_xlsb = (uint32_t)reg_data[1] << 8;
    data_lsb = (uint32_t)reg_data[2];
    data->bpm32 = data_msb | data_xlsb | data_lsb;
    data->bpm = (double)data->bpm32 / 16384.0;
}

/*!
 * @brief This API process data buffer to get BPM value.
 */
uint8_t max30102_get_bpm(int32_t *data)
{
    uint8_t i;
    uint32_t avg;
    long acc;
    uint8_t crossSample = MAX30102_BPM_NO_SAMPLES;
    uint8_t afterCrossSample = MAX30102_BPM_NO_SAMPLES;
    uint8_t delaySamples = 0;
    double periodAvg;

    acc = 0;
    avg = 0;
    for (i = 0; i < MAX30102_BPM_SAMPLES_SIZE; i++){
        acc += data[i];
    }
    avg = acc / MAX30102_BPM_SAMPLES_SIZE;

    for (i = 0; i < MAX30102_BPM_SAMPLES_SIZE; i++){
        data[i] -= avg;
    }

    for (i = 0; i < (MAX30102_BPM_SAMPLES_SIZE - 1); i++){
        if (crossSample == MAX30102_BPM_NO_SAMPLES && data[i] > 0 && data[i+1] <= 0){
            crossSample = i;
            continue;
        }

        if (crossSample != MAX30102_BPM_NO_SAMPLES){
            if (afterCrossSample == MAX30102_BPM_NO_SAMPLES && data[i] > 0 && data[i+1] <= 0){
                afterCrossSample = i;
            }
        }

        if (crossSample != MAX30102_BPM_NO_SAMPLES && afterCrossSample != MAX30102_BPM_NO_SAMPLES){
            delaySamples = afterCrossSample - crossSample;
            break;
        }
    }

    periodAvg = 0.04 * (double)delaySamples;

    return (uint8_t)(60.0 / periodAvg);
}

/*!
 * @brief This internal API interleaves the register address between the
 * register data buffer for burst write operation.
 */
static void interleave_reg_addr(const uint8_t *reg_addr, uint8_t *temp_buff, const uint8_t *reg_data, uint8_t len)
{
    uint8_t index;

    for (index = 1; index < len; index++)
    {
        temp_buff[(index * 2) - 1] = reg_addr[index];
        temp_buff[index * 2] = reg_data[index];
    }
}

/*!
 * @brief This internal API is used to validate the device structure pointer for
 * null conditions.
 */
static int8_t null_ptr_check(const struct max30102_dev *dev)
{
    int8_t rslt;

    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_us == NULL))
    {
        /* Device structure pointer is not valid */
        rslt = MAX30102_E_NULL_PTR;
    }
    else
    {
        /* Device structure is fine */
        rslt = MAX30102_OK;
    }

    return rslt;
}
