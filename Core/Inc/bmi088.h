//
// Created by MSI on 2025/10/12.
//

#ifndef SPITEST_BMI088_H
#define SPITEST_BMI088_H
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"
#ifdef __cplusplus
extern "C" {
#endif
void bmi088_write_byte(uint8_t tx_data);
void bmi088_read_byte(uint8_t *rx_data, uint8_t length);
void bmi088_write_reg(uint8_t reg, uint8_t data);
void BMI088_ACCEL_NS_L(void);
void BMI088_ACCEL_NS_H(void);
void BMI088_GYRO_NS_L(void);
void BMI088_GYRO_NS_H(void);
void bmi088_init(void);
void bmi088_accel_write_single_reg(uint8_t reg, uint8_t data);
void bmi088_accel_read_reg(uint8_t reg, uint8_t *rx_data, uint8_t length); // 加速度计读取，注意需要忽略第一位数据dummy byte
void bmi088_gyro_read_reg(uint8_t reg, uint8_t *rx_data, uint8_t length);// 陀螺仪读取
void bmi088_gyro_write_single_reg(uint8_t reg, uint8_t tx_data); // gyro写入

#ifdef __cplusplus
}
#endif


#endif // SPITEST_BMI088_H
