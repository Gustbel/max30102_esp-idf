#include <stdio.h>
#include "MAX30102_ESP32C3.h"

/**
    @brief Read MAX30102 through I2C port

    @param reg_addr The address of the first register to read.
    @param reg_data The buffer to store the data retrieved by MAX30102.
    @param len The number of positions to read.
    @param intf_ptr Not used. Keep it for signature compatibility.

    @attention This function is platform-specific. For a trust data access, please use the drivers functions.
*/
MAX30102_INTF_RET_TYPE esp32c3_read_max30102(uint8_t reg_addr, uint8_t *reg_data, uint32_t len)
{
    MAX30102_INTF_RET_TYPE ret;
    int i;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MAX30102_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MAX30102_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);

    for (i = 0; i < (len - 1); i++)
    {
        i2c_master_read_byte(cmd, reg_data + i, ACK_VAL);
    }

    i2c_master_read_byte(cmd, reg_data + i, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

/**
 *  @brief Write MAX30102 register through I2C port
 *  
 *  @param reg_addr The address of the first register to write.
 *  @param reg_data The buffer that stores the data to be written.
 *  @param len The number of positions to write.
 *  @param intf_ptr Not used. Keep it in order to maintain signature compatibility.
 * 
 *  @attention This function is platform-specific. For a trust data access, please use the drivers functions.
*/
MAX30102_INTF_RET_TYPE esp32c3_write_max30102(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len)
{

    MAX30102_INTF_RET_TYPE ret;
    int i;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MAX30102_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);

    for (i = 0; i < len; i++)
    {
        i2c_master_write_byte(cmd, reg_addr + i, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, *(reg_data + i), ACK_CHECK_EN);
    }

    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

/**
 *  @brief Wait for a specific amount of us
 *  
 *  @param period The number of microseconds to wait.
 * 
 *  @attention This function is platform-specific. For a trust data access, please use the drivers functions.
 */
void esp32c3_delay_us_max30102(uint32_t period)
{
    vTaskDelay((period / 1000) / portTICK_RATE_MS);
}
