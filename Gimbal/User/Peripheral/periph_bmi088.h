/*
 * @Project: Infantry Code
 *
 * @Author: GDDG08
 * @Date: 2021-12-31 17:37:14
 * @LastEditors: Hatrix 3113624526@qq.com
 * @LastEditTime: 2024-02-12 16:45:09
 */

#ifndef PERIPH_BMI088_H
#define PERIPH_BMI088_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "BMI088reg.h"
#include "util_spi.h"
#include "util_gpio.h"
#include "lib_buff.h"
#include "lib_math.h"
#include "sys_dwt.h"

#define GRAVITY 9.81f
#define BMI088_OFFLINE_TIME 0.3f

#define IF_CAILBRATE 0
#define GXOFFSET 0.000658329518f
#define GYOFFSET -0.00570373517f
#define GZOFFSET  -0.00247374992f
#define GNORM 9.69293118f

#define BMI088_ACCEL_WRITE_SINGLE_REG(reg, data, hspi, gpio_accel) \
    {                                                              \
        GPIO_Reset(gpio_accel);                                    \
        SPI_SwapAbyte(hspi, reg);                                  \
        SPI_SwapAbyte(hspi, data);                                 \
        GPIO_Set(gpio_accel);                                      \
    }
#define BMI088_ACCEL_READ_SINGLE_REG(reg, data, hspi, gpio_accel) \
    {                                                             \
        GPIO_Reset(gpio_accel);                                   \
        SPI_SwapAbyte(hspi, ((reg) | 0x80));                      \
        SPI_SwapAbyte(hspi, 0x55);                                \
        (data) = SPI_SwapAbyte(hspi, 0x55);                       \
        GPIO_Set(gpio_accel);                                     \
    }
#define BMI088_ACCEL_READ_MULI_REG(reg, data, len, hspi, gpio_accel) \
    {                                                                \
        GPIO_Reset(gpio_accel);                                      \
        SPI_SwapAbyte(hspi, ((reg) | 0x80));                         \
        SPI_SwapAbyte(hspi, ((reg) | 0x80));                         \
        SPI_ReadMuliReg(hspi, data, len);                            \
        GPIO_Set(gpio_accel);                                        \
    }
#define BMI088_GYRO_WRITE_SINGLE_REG(reg, data, hspi, gpio_gyro) \
    {                                                            \
        GPIO_Reset(gpio_gyro);                                   \
        SPI_SwapAbyte(hspi, reg);                                \
        SPI_SwapAbyte(hspi, data);                               \
        GPIO_Set(gpio_gyro);                                     \
    }
#define BMI088_GYRO_READ_SINGLE_REG(reg, data, hspi, gpio_gyro) \
    {                                                           \
        GPIO_Reset(gpio_gyro);                                  \
        SPI_SwapAbyte(hspi, ((reg) | 0x80));                    \
        (data) = SPI_SwapAbyte(hspi, 0x55);                     \
        GPIO_Set(gpio_gyro);                                    \
    }
#define BMI088_GYRO_READ_MULI_REG(reg, data, len, hspi, gpio_gyro) \
    {                                                              \
        GPIO_Reset(gpio_gyro);                                     \
        SPI_SwapAbyte(hspi, ((reg) | 0x80));                       \
        SPI_ReadMuliReg(hspi, data, len);                          \
        GPIO_Set(gpio_gyro);                                       \
    }

    typedef enum
    {
        BMI088_STATE_NULL = 0,
        BMI088_STATE_CONNECTED = 1,
        BMI088_STATE_LOST = 2,
        BMI088_STATE_ERROR = 3,
        BMI088_STATE_PENDING = 4
    } BMI088_StateEnum;

    typedef struct
    {
        float x;
        float y;
        float z;
    } BMI088_AccelTypeDef;

    typedef struct
    {
        float yaw;
        float pitch;
        float row;
    } BMI088_SpeedTypeDef;

    typedef struct
    {
        BMI088_StateEnum state;

        SPI_HandleTypeDef *hspi;
        GPIO_HandleTypeDef *gpio_accel;
        GPIO_HandleTypeDef *gpio_gyro;

        BMI088_AccelTypeDef accel; //  real data
        BMI088_SpeedTypeDef gyro;  //  real data

        float d_accel[3];
        float d_gyro[3];

        float gyro_offset[3];
        float accel_offset[3];
        float gNorm;
        float accelScale;
        float sensor_time;
        float temperature;

        uint8_t caliOffset;
        float BMI088_ACCEL_SEN;
        float BMI088_GYRO_SEN;
        uint8_t calibrate;

        float update_dt;
        uint32_t last_update_tick;
    } BMI088_DataTypeDef;

    void BMI088_Decode(BMI088_DataTypeDef *bmi088);
    void BMI088_SetOffset(BMI088_DataTypeDef *bmi088);
    void BMI088_GetOffset(BMI088_DataTypeDef *bmi088);

    uint8_t BMI088_Init(BMI088_DataTypeDef *bmi088, SPI_HandleTypeDef *hspi, GPIO_HandleTypeDef *gpio_accel, GPIO_HandleTypeDef *gpio_gyro);
    void BMI088_Reset(BMI088_DataTypeDef *bmi088);
    static uint8_t BMI088_AccelInit(BMI088_DataTypeDef *bmi088);
    static uint8_t BMI088_GyroInit(BMI088_DataTypeDef *bmi088);
    static uint8_t BMI088_AccelTest(BMI088_DataTypeDef *bmi088);
    static uint8_t BMI088_GyroTest(BMI088_DataTypeDef *bmi088);
    uint8_t BMI088_IsLost(BMI088_DataTypeDef *bmi088);

#endif

#ifdef __cplusplus
}
#endif
