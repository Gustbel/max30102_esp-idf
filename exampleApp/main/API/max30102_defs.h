#ifndef MAX30102_DEFS_H_
#define MAX30102_DEFS_H_

/********************************************************/
#include <stdint.h>
#include <stddef.h>

/********************************************************/
/*! @name       Common macros               */
/********************************************************/

#if !defined(UINT8_C) && !defined(INT8_C)
#define INT8_C(x)    S8_C(x)
#define UINT8_C(x)   U8_C(x)
#endif

#if !defined(UINT16_C) && !defined(INT16_C)
#define INT16_C(x)   S16_C(x)
#define UINT16_C(x)  U16_C(x)
#endif

#if !defined(INT32_C) && !defined(UINT32_C)
#define INT32_C(x)   S32_C(x)
#define UINT32_C(x)  U32_C(x)
#endif

#if !defined(INT64_C) && !defined(UINT64_C)
#define INT64_C(x)   S64_C(x)
#define UINT64_C(x)  U64_C(x)
#endif

/**@}*/
/**\name C standard macros */
#ifndef NULL
#ifdef __cplusplus
#define NULL         0
#else
#define NULL         ((void *) 0)
#endif
#endif

/********************************************************/
#ifndef TRUE
#define TRUE                                      UINT8_C(1)
#endif
#ifndef FALSE
#define FALSE                                     UINT8_C(0)
#endif

/**
 * MAX30102_INTF_RET_TYPE is the read/write interface return type which can be overwritten by the build system.
 */
#ifndef MAX30102_INTF_RET_TYPE
#define MAX30102_INTF_RET_TYPE                      int8_t
#endif

/**
 * The last error code from read/write interface is stored in the device structure as intf_rslt.
 */
#ifndef MAX30102_INTF_RET_SUCCESS
#define MAX30102_INTF_RET_SUCCESS                   INT8_C(0)
#endif

/**\name MAX30102 chip identifier */
#define MAX30102_CHIP_ID                            UINT8_C(0x15)

/**\name Register Address */
#define MAX30102_INT_STATUS1_ADDR               UINT8_C(0x00)
#define MAX30102_INT_STATUS2_ADDR               UINT8_C(0x01)
#define MAX30102_INT_ENABLE1_ADDR               UINT8_C(0x02)
#define MAX30102_INT_ENABLE2_ADDR               UINT8_C(0x03)
#define MAX30102_FIFO_WR_PTR_ADDR               UINT8_C(0x04)
#define MAX30102_FIFO_OVF_CTR_ADDR              UINT8_C(0x05)
#define MAX30102_FIFO_RD_PTR_ADDR               UINT8_C(0x06)
#define MAX30102_FIFO_DATA_ADDR                 UINT8_C(0x07)
#define MAX30102_FIFO_CFG_ADDR                  UINT8_C(0x08)
#define MAX30102_MODE_CFG_ADDR                  UINT8_C(0x09)
#define MAX30102_SPO2_CFG_ADDR                  UINT8_C(0x0A)
/* 0x0B is reserved */
#define MAX30102_LED1_PA_ADDR                   UINT8_C(0x0C)
#define MAX30102_LED2_PA_ADDR                   UINT8_C(0x0D)
/* 0x0E and 0x0F are reserved */
#define MAX30102_SLOT_1_2_ADDR                  UINT8_C(0x11)
#define MAX30102_SLOT_3_4_ADDR                  UINT8_C(0x12)
/* 0x13 trough 0x1E are reserved */
#define MAX30102_TEMP_INT_ADDR                  UINT8_C(0x1F)
#define MAX30102_TEMP_FRAC_ADDR                 UINT8_C(0x20)
#define MAX30102_TEMP_CFG_ADDR                  UINT8_C(0x21)

#define MAX30102_REV_ID_ADDR                    UINT8_C(0xFE)
#define MAX30102_CHIP_ID_ADDR                   UINT8_C(0xFF)

/**\name Interrupt status flags */
/*! @todo Define int status flags */

/**\name Interrupt enable flags */
/*! @todo Define int enable flags */

/**\name FIFO configuration values */
/* Samples averaging */
#define MAX30102_SMP_AVE_NO                     UINT8_C(0x00)
#define MAX30102_SMP_AVE_2                      UINT8_C(0x20)
#define MAX30102_SMP_AVE_4                      UINT8_C(0x40)
#define MAX30102_SMP_AVE_8                      UINT8_C(0x60)
#define MAX30102_SMP_AVE_16                     UINT8_C(0x80)
#define MAX30102_SMP_AVE_32                     UINT8_C(0xE0)

/* Roll over enable */
#define MAX30102_FIFO_ROLL_OVER                 UINT8_C(0x10)

/**\name MODE configuration */
#define MAX30102_SHDWN                          UINT8_C(0x80)
#define MAX30102_RESET                          UINT8_C(0x40)
#define MAX30102_HR_MODE                        UINT8_C(0x02)
#define MAX30102_SPO2_MODE                      UINT8_C(0x03)
#define MAX30102_MULTILED_MODE                  UINT8_C(0x07)

/**\name SPO2 configuration */
#define MAX30102_SPO2_RANGE_2048                UINT8_C(0x00)
#define MAX30102_SPO2_RANGE_4096                UINT8_C(0x20)
#define MAX30102_SPO2_RANGE_8192                UINT8_C(0x40)
#define MAX30102_SPO2_RANGE_16384               UINT8_C(0x60)
#define MAX30102_SPO2_50_SPS                    UINT8_C(0x00)
#define MAX30102_SPO2_100_SPS                   UINT8_C(0x04)
#define MAX30102_SPO2_200_SPS                   UINT8_C(0x08)
#define MAX30102_SPO2_400_SPS                   UINT8_C(0x0C)
#define MAX30102_SPO2_800_SPS                   UINT8_C(0x10)
#define MAX30102_SPO2_1000_SPS                  UINT8_C(0x14)
#define MAX30102_SPO2_1600_SPS                  UINT8_C(0x18)
#define MAX30102_SPO2_3200_SPS                  UINT8_C(0x1C)
#define MAX30102_SPO2_LED_PW_69                 UINT8_C(0x00)
#define MAX30102_SPO2_LED_PW_118                UINT8_C(0x01)
#define MAX30102_SPO2_LED_PW_215                UINT8_C(0x02)
#define MAX30102_SPO2_LED_PW_411                UINT8_C(0x03)

/**\name Multiled mode configuration */
/*! @todo Define configuration values for multiled mode. */

/**\name API success code */
#define MAX30102_OK                                 INT8_C(0)

/**\name API error codes */
#define MAX30102_E_NULL_PTR                         INT8_C(-1)
#define MAX30102_E_DEV_NOT_FOUND                    INT8_C(-2)
#define MAX30102_E_INVALID_LEN                      INT8_C(-3)
#define MAX30102_E_COMM_FAIL                        INT8_C(-4)
#define MAX30102_E_SLEEP_MODE_FAIL                  INT8_C(-5)
#define MAX30102_E_NVM_COPY_FAILED                  INT8_C(-6)

// /**\name Macros related to size */
#define MAX30102_DATA_LEN                           UINT8_C(6)

/**\name Macro to combine two 8 bit data's to form a 16 bit data */
#define BME280_CONCAT_BYTES(msb, lsb)             (((uint16_t)msb << 8) | (uint16_t)lsb)

#define BME280_SET_BITS(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MSK)) | \
     ((data << bitname##_POS) & bitname##_MSK))
#define BME280_SET_BITS_POS_0(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MSK)) | \
     (data & bitname##_MSK))

#define BME280_GET_BITS(reg_data, bitname)        ((reg_data & (bitname##_MSK)) >> \
                                                   (bitname##_POS))
#define BME280_GET_BITS_POS_0(reg_data, bitname)  (reg_data & (bitname##_MSK))

/**\name Sensor component selection macros
 * These values are internal for API implementation.
 */
#define MAX30102_BPM                              UINT8_C(1)
#define MAX30102_ALL                                UINT8_C(0x07)

// /**\name Settings selection macros */
#define MAX30102_BPM_SAMPLES_SIZE 50 /*!< The size of the buffer for the samples from sensor */
#define MAX30102_BPM_PERIOD_SAMPLE_SIZE 4 /*!< The size of the buffer for average period */
#define MAX30102_BPM_NO_SAMPLES MAX30102_BPM_SAMPLES_SIZE + 1 /*!< Dummy value for zero-crossing detection indexes */

/*!
 * @brief Interface selection Enums
 */
enum max30102_intf {
    /*< SPI interface */
    MAX30102_SPI_INTF,
    /*< I2C interface */
    MAX30102_I2C_INTF
};

/*!
 * @brief Type definitions
 */

/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific read functions of the user
 *
 * @param[in] reg_addr       : Register address from which data is read.
 * @param[out] reg_data     : Pointer to data buffer where read data is stored.
 * @param[in] len            : Number of bytes of data to be read.
 * @param[in, out] intf_ptr  : Void pointer that can enable the linking of descriptors
 *                                  for interface related call backs.
 *
 * @retval   0 -> Success.
 * @retval Non zero value -> Fail.
 *
 */
typedef MAX30102_INTF_RET_TYPE (*max30102_read_fptr_t)(uint8_t reg_addr, uint8_t *reg_data, uint32_t len);

/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific write functions of the user
 *
 * @param[in] reg_addr      : Register address to which the data is written.
 * @param[in] reg_data     : Pointer to data buffer in which data to be written
 *                            is stored.
 * @param[in] len           : Number of bytes of data to be written.
 * @param[in, out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                            for interface related call backs
 *
 * @retval   0   -> Success.
 * @retval Non zero value -> Fail.
 *
 */
typedef MAX30102_INTF_RET_TYPE (*max30102_write_fptr_t)(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len);

/*!
 * @brief Delay function pointer which should be mapped to
 * delay function of the user
 *
 * @param[in] period              : Delay in microseconds.
 * @param[in, out] intf_ptr       : Void pointer that can enable the linking of descriptors
 *                                  for interface related call backs
 *
 */
typedef void (*max30102_delay_us_fptr_t)(uint32_t period);

struct max30102_data
{
    uint32_t bpm32;
    double bpm;
};

/*!
 * @brief MAX30102 sensor settings structure.
 */
struct max30102_settings
{
    uint8_t fifo;
    uint8_t spo2;
    uint8_t multiled;
};

/*!
 * @brief max30102 device structure
 */
struct max30102_dev
{
    /*< Chip Id */
    uint8_t chip_id;

    /*< Interface function pointer used to enable the device address for I2C and chip selection for SPI */
    void *intf_ptr;

    /*< Interface Selection
     * For SPI, intf = MAX30102_SPI_INTF
     * For I2C, intf = MAX30102_I2C_INTF
     * */
    // enum max30102_intf intf;

    /*< Read function pointer */
    max30102_read_fptr_t read;

    /*< Write function pointer */
    max30102_write_fptr_t write;

    /*< Delay function pointer */
    max30102_delay_us_fptr_t delay_us;

    /*< Sensor settings */
    struct max30102_settings settings;

    /*< Variable to store result of read/write function */
    MAX30102_INTF_RET_TYPE intf_rslt;
};

#endif /* MAX30102_DEFS_H_ */
