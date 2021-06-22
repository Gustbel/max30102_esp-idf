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

#define IIR_COEFF_SIZE 3

SemaphoreHandle_t print_mux = NULL;

/* The following data structures are used to interact with MAX30102 */
static MAX30102_DEVICE device;
static MAX30102_DATA mess_data;
static double samples[MAX30102_BPM_SAMPLES_SIZE];
static double samplesGrad[MAX30102_BPM_SAMPLES_SIZE];
static double signal[IIR_COEFF_SIZE] = {0.0};
static double outputNS[IIR_COEFF_SIZE] = {0.0};

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
    uint8_t bpmBuffer[MAX30102_BPM_PERIOD_SAMPLE_SIZE];

    uint8_t bpmTickCnt = 0;
    double bpmAvg = 0.0;
    double lastBpm = 0.0;
    uint8_t lastSampleIdx = MAX30102_BPM_SAMPLES_SIZE;
    uint8_t firstSampleIdx = MAX30102_BPM_SAMPLES_SIZE;

    const uint8_t bpmAvgSize = 4;
    int cnt = 0, i, j;

    const double a[IIR_COEFF_SIZE] = {
        1.00000,
        -0.74779,
        0.27221
    };

    const double b[IIR_COEFF_SIZE] = {
        0.13111,
        0.26221,
        0.13111
    };

    const double sat_max = 250.0;
    const double sat_min = 0.0;

    
    double samplesMax = 0.0;

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

        /* FIFO reset for new adquisition */
        reg_addr = MAX30102_FIFO_WR_PTR_ADDR;
        setup = 0x00;
        ret = max30102_set_regs(&reg_addr, &setup, 1, &device);
        device.delay_us(4000);
        reg_addr = MAX30102_FIFO_RD_PTR_ADDR;
        ret = max30102_set_regs(&reg_addr, &setup, 1, &device);
        device.delay_us(4000);
        reg_addr = MAX30102_FIFO_OVF_CTR_ADDR;
        ret = max30102_set_regs(&reg_addr, &setup, 1, &device);
        device.delay_us(4000);

        /* Clear signal vector */
        for (i = 0; i < MAX30102_BPM_SAMPLES_SIZE; i++){
            samples[i] = 0.0;
        }

        /* Setup adquisition mode and wait for a new vector */
        ret = max30102_set_sensor_mode(MAX30102_SPO2_MODE, &device);
        device.delay_us(700000);
    
        /* Get samples from sensor into signal vector */
        for (i = 0; i < MAX30102_BPM_SAMPLES_SIZE; i++){
            ret = max30102_get_sensor_data(MAX30102_BPM, &mess_data, &device);
            samples[i] = mess_data.bpmR;
            device.delay_us(21000);
        }

        /* IIR */
        for (i = 0; i < IIR_COEFF_SIZE; i++){
            signal[i] = 0.0;
            outputNS[i] = 0.0;
        }
        for (i = 0; i < MAX30102_BPM_SAMPLES_SIZE; i++){
            for (j = IIR_COEFF_SIZE - 1; j; j--){
                signal[j] = signal[j-1];
                outputNS[j] = outputNS[j-1];
            }
            outputNS[0] = 0.0;
            signal[0] = samples[i];
            for (j = 0; j < IIR_COEFF_SIZE; j++){
                outputNS[0] += b[j] * signal[j];
            }
            for (j = 1; j < IIR_COEFF_SIZE; j++){
                outputNS[0] -= a[j] * outputNS[j];
            }
            outputNS[0] -= a[0];
            
            if (outputNS[0] > sat_max) outputNS[0] = sat_max;
            else if (outputNS[0] < sat_min) outputNS[0] = sat_min;
            samples[i] = outputNS[0];
        }

        /* Gradient detection */
        for (i = 0; i < MAX30102_BPM_SAMPLES_SIZE - 1; i++){
            samplesGrad[i] = samples[i] - samples[i+1];
            if (samplesGrad[i] < 0.0) samplesGrad[i] = 0.0;
        }
        samplesGrad[i] = 0.0;

        /* Get the max value */
        samplesMax = 0.0;
        for (i = 0; i < MAX30102_BPM_SAMPLES_SIZE; i++){
            if (samplesGrad[i] > samplesMax) samplesMax = samplesGrad[i];
        }

        /* Clear ticks vector */
        for (i = 0; i < MAX30102_BPM_PERIOD_SAMPLE_SIZE; i++){
            bpmBuffer[i] = 0;
        }

        /* Populate ticks vector from gradient detection vector */
        for (i = 0, j = 0; i < MAX30102_BPM_SAMPLES_SIZE && j < MAX30102_BPM_PERIOD_SAMPLE_SIZE; i++){
            if (samplesGrad[i] > (0.7 * samplesMax)){
                bpmBuffer[j++] = i;
                i += 5;
            }
        }

        /* From ticks vector, get average period and period number */
        bpmTickCnt = 0;
        bpmAvg = 0.0;
        for (i = 1; i < MAX30102_BPM_PERIOD_SAMPLE_SIZE; i++){
            if (bpmBuffer[i] != 0){
                bpmAvg += (double)bpmBuffer[i] - (double)bpmBuffer[i-1];
                bpmTickCnt++;
            }
        }

        /* If there is ticks detected, calculate the BPM */
        if (bpmTickCnt > 0){
            bpmAvg /= (double)bpmTickCnt;
            bpmAvg *= 0.02;
            bpmAvg = 60.0 / bpmAvg;
            lastBpm = bpmAvg;
        }

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

            // Enable this in order to get the samples and
            // gradient vector elements printed out in the screen.
#ifdef SAMPLES_DEBUG
            for (i = 0; i < MAX30102_BPM_SAMPLES_SIZE; i++){
                printf("%d,%.1f,%.1f\n", i, samples[i], samplesGrad[i]);
            }
            printf("%d Ticks at: ", bpmTickCnt);
            for (i = 0; i < MAX30102_BPM_PERIOD_SAMPLE_SIZE; i++){
                printf("%d, ", bpmBuffer[i]);
            }
#endif
            printf("\nBPM: %.1f\n", bpmAvg);
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
    ret = max30102_set_fifo(MAX30102_SMP_AVE_NO, &device);
    device.delay_us(40000);

    reg_addr = MAX30102_SLOT_1_2_ADDR;
    setup = 0x02;
    ret = max30102_set_regs(&reg_addr, &setup, 1, &device);
    device.delay_us(40000);

    setup = 0x00;
    reg_addr = MAX30102_SLOT_3_4_ADDR;
    ret = max30102_set_regs(&reg_addr, &setup, 1, &device);

    // Setup the LEDs current amplitude
    // - Aprox. 3 mA
    // ret = max30102_set_led_amplitude(0x01, &device);
    reg_addr = MAX30102_LED1_PA_ADDR;
    setup = 0x24;
    ret = max30102_set_regs(&reg_addr, &setup, 1, &device);
    device.delay_us(40000);
    reg_addr = MAX30102_LED2_PA_ADDR;
    setup = 0x24;
    ret = max30102_set_regs(&reg_addr, &setup, 1, &device);

    // For this example there is only one task which setup the MAX30102, and then
    // reads the parameters periodically.
    xTaskCreate(i2c_test_task, "i2c_test_task_0", 1024 * 2, (void *)0, 10, NULL);
}
