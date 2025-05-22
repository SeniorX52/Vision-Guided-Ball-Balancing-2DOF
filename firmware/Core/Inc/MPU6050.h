/*
 * MPU6050.h
 *
 *  Created on: Aug 16, 2024
 *      Author: biffb
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_


#include "stm32f1xx_hal.h" // Adjust according to your STM32 series

#define MPU6050_ADDRESS         (0x68 << 1)  // MPU6050 I2C address
#define MPU6050_WHO_AM_I        0x75
#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_ACCEL_XOUT_H    0x3B
#define MPU6050_GYRO_XOUT_H     0x43
#define MPU6050_ACCEL_CONFIG    0x1C
#define MPU6050_GYRO_CONFIG     0x1B
#define MPU6050_CONFIG			0x1A

typedef struct {
    I2C_HandleTypeDef *hi2c;
} MPU6050_HandleTypeDef;

HAL_StatusTypeDef MPU6050_Init(MPU6050_HandleTypeDef *mpu);
HAL_StatusTypeDef MPU6050_ReadAccel(MPU6050_HandleTypeDef *mpu, int16_t* accelData);
HAL_StatusTypeDef MPU6050_ReadGyro(int16_t* gyroData);
HAL_StatusTypeDef MPU6050_WriteRegister(MPU6050_HandleTypeDef *mpu, uint8_t reg, uint8_t data);
HAL_StatusTypeDef MPU6050_ReadRegister(MPU6050_HandleTypeDef *mpu, uint8_t reg, uint8_t* data);
void MPU6050_Calibrate(void);
void ComputeAccelAngles(int16_t ax, int16_t ay, int16_t az, float* pitch, float* roll);
void MPU6050_CalculatePitchRoll(float* pitch, float* roll);
void MPU6050_resetPitchRollOffset();

#endif /* INC_MPU6050_H_ */
