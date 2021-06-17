/* MAX30102 example application

This example shows the basic use (configuration, initialization, read) of the
MAX30102 API

*/
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "API/max30102.h"

static const char *TAG = "exampleApp";

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA               /*!< gpio number for I2C master data  */

#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */


SemaphoreHandle_t print_mux = NULL;

/* The following data structures are used to interact with MAX30102 */
static MAX30102_DEVICE device;
static MAX30102_DATA mess_data;
static int32_t samples[MAX30102_BPM_SAMPLES_SIZE];

//
// I2C master initialization
//
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };
    
    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    
    if (err != ESP_OK) 
    {
        return err;
    }

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

//
// Test task
//
static void i2c_test_task(void *arg)
{
    uint8_t ret = MAX30102_OK;
    uint8_t reg_addr, setup;
    uint32_t task_idx = (uint32_t)arg;
    uint8_t bpmBuffer[8];
    uint8_t bpmIdx = 0;
    uint32_t bpmAvg = 0;
    const uint8_t bpmAvgSize = 4;
    int cnt = 0, i;

    // Here is the main loop. Periodically reads and print the parameters
    // measured from MAX30102.
    while (1) 
    {
        ESP_LOGI(TAG, "T: %d test #: %d", task_idx, cnt++);

        // Setup the sensor operation mode for heart rate, wait for a 700 ms
        // and then, read the data buffer. The read is made by
        // passing the mess_data struct, which contains the
        // meas result.
        // After getting the values, get a semaphore in order to
        // print the information for monitoring

        reg_addr = MAX30102_FIFO_WR_PTR_ADDR;
        setup = 0x00;
        ret = max30102_set_regs(&reg_addr, &setup, 1, &device);
        device.delay_us(40000);
        reg_addr = MAX30102_FIFO_RD_PTR_ADDR;
        ret = max30102_set_regs(&reg_addr, &setup, 1, &device);
        device.delay_us(4000);
        reg_addr = MAX30102_FIFO_OVF_CTR_ADDR;
        ret = max30102_set_regs(&reg_addr, &setup, 1, &device);
        device.delay_us(40000);

        ret = max30102_set_sensor_mode(MAX30102_MULTILED_MODE, &device);
        device.delay_us(700000);
    
        for (i = 0; i < MAX30102_BPM_SAMPLES_SIZE; i++){
            ret = max30102_get_sensor_data(MAX30102_BPM, &mess_data, &device);
            samples[i] = mess_data.bpm32;
            device.delay_us(20000);
        }

        // Once the data buffer is full, we need to get the BPM measurement.
        // In order to improve performance, the BPM mess is filtered (mean)
        // 
        bpmAvg = 0;
        for (i = bpmAvgSize - 1; i; i--){
            bpmBuffer[i] = bpmBuffer[i-1];
        }

        bpmBuffer[0] = max30102_get_bpm(samples);
        for (i = 0; i < bpmAvgSize; i++){
            bpmAvg += bpmBuffer[i];
        }

        bpmAvg /= bpmAvgSize;

        xSemaphoreTake(print_mux, portMAX_DELAY);

        // Print information retrieved. If the connection was successful, print
        // the sensor ID and the BPM average value.
        // If the connection is NOK, print an error message.
        if (ret == MAX30102_OK) 
        {
            printf("***********************************\n");
            printf("T: %d -  READING SENSOR( MAX30102 )\n", task_idx);
            printf("***********************************\n\n");
            printf("Print direct values:\n");
            printf("Sensor ID: %p\n", device.chip_id);
            printf("BPM: %d \n", bpmAvg);
        } 
        else 
        {
            ESP_LOGW(TAG, "%s: No ack, sensor not connected...skip...", esp_err_to_name(ret));
        }

        // Give the semaphore taken and wait for the next read.
        xSemaphoreGive(print_mux);
        device.delay_us(1000000);
    }

    vSemaphoreDelete(print_mux);
    vTaskDelete(NULL);
}

//
// This is the entry point for the example application. This shows how to
// use the MAX30102 API functions and data structures.
//
void app_main(void)
{
    int8_t ret;
    uint8_t reg_addr;
    uint8_t setup;
    print_mux = xSemaphoreCreateMutex();

    // Before access to the MAX30102, we need to setup the device handler
    // by assign the platform specific functions which brings access
    // to the communication port. These functions are implemented in
    // API/driver/MAX30102_ESP32C3.c file.
    device.read = esp32c3_read_max30102;
    device.write = esp32c3_write_max30102;
    device.delay_us = esp32c3_delay_us_max30102;

    // The first step is to initialize the I2C peripheral as usual
    ESP_ERROR_CHECK(i2c_master_init());

    // After I2C initialization, BME280 initialization could be done.
    if (MAX30102_E_DEV_NOT_FOUND == max30102_init(&device))
    {
        ESP_LOGW(TAG, "MAX30102 sensor is not connected.");
        while(1);
    }

    // Setup some adquisition parameters
    // - ADC range for 16384 counts.
    // - 50 samples per second.
    // - 411 microseconds pulse width for LEDs.
    ret = max30102_set_spo2(MAX30102_SPO2_RANGE_16384 | MAX30102_SPO2_50_SPS | MAX30102_SPO2_LED_PW_411, &device);
    device.delay_us(40000);

    // Setup the FIFO
    // - No samples average.
    ret = max30102_set_fifo(MAX30102_SMP_AVE_2, &device);
    device.delay_us(40000);

    reg_addr = MAX30102_SLOT_1_2_ADDR;
    setup = 0x01;
    ret = max30102_set_regs(&reg_addr, &setup, 1, &device);
    device.delay_us(40000);

    reg_addr = MAX30102_SLOT_3_4_ADDR;
    ret = max30102_set_regs(&reg_addr, &setup, 1, &device);

    // Setup the LEDs current amplitude
    // - Aprox. 3 mA
    // ret = max30102_set_led_amplitude(0x01, &device);
    reg_addr = MAX30102_LED1_PA_ADDR;
    setup = 0x7F;
    ret = max30102_set_regs(&reg_addr, &setup, 1, &device);
    device.delay_us(40000);
    reg_addr = MAX30102_LED2_PA_ADDR;
    ret = max30102_set_regs(&reg_addr, &setup, 1, &device);

    // For this example there is only one task which setup the MAX30102, and then
    // reads the parameters periodically.
    xTaskCreate(i2c_test_task, "i2c_test_task_0", 1024 * 2, (void *)0, 10, NULL);
}
