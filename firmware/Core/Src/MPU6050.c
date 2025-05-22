/*
 * MPU6050.c
 *
 *  Created on: Aug 16, 2024
 *      Author: biffb
 */

#include "MPU6050.h"
#include <math.h>
static float pitch_offset = 0;
static float roll_offset = 0;
static float prev_pitch = 0;
static float prev_roll = 0;
static int16_t gx_offset = 0;
static int16_t gy_offset = 0;
static int16_t gz_offset = 0;
#define ACCEL_SCALE_FACTOR 8192.0f
#define GYRO_SCALE_FACTOR 131.0f

MPU6050_HandleTypeDef* mpuHandle;

// Computes the pitch and roll angles based on accelerometer data
void ComputeAccelAngles(int16_t ax, int16_t ay, int16_t az, float* pitch, float* roll) {
    float ax_g = (float)ax / ACCEL_SCALE_FACTOR ;
    float ay_g = (float)ay / ACCEL_SCALE_FACTOR ;
    float az_g = (float)az / ACCEL_SCALE_FACTOR ;

    *pitch = atan2(ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * (180.0f / M_PI)-pitch_offset;
    *roll = atan2(ay_g, sqrt(ax_g * ax_g + az_g * az_g)) * (180.0f / M_PI)-roll_offset;

    *roll = fmaxf(fminf(*roll, 10.0f), -10.0f);
    *pitch = fmaxf(fminf(*pitch, 10.0f), -10.0f);
    if (fabsf(*roll) < 3.0f) *roll = 0.0f;
    if (fabsf(*pitch) < 3.0f) *pitch = 0.0f;
    *pitch =  0.5 * prev_pitch + (1 - 0.5) * (*pitch);
    prev_pitch=*pitch;
    *roll= 0.5 * prev_pitch + (1 - 0.5) * (*roll);
    prev_roll=*roll;

}

// Calculates the pitch and roll angles using the MPU6050 accelerometer data
void MPU6050_CalculatePitchRoll(float* pitch, float* roll){
	int16_t accelerometer[3]={0,0,0};
	MPU6050_ReadAccel(mpuHandle, accelerometer);
	ComputeAccelAngles(accelerometer[0], accelerometer[1], accelerometer[2],pitch, roll);
}
//Initialize the sensor
HAL_StatusTypeDef MPU6050_Init(MPU6050_HandleTypeDef *mpu) {
    HAL_StatusTypeDef ret;
    mpuHandle=mpu;
    ret = MPU6050_WriteRegister(mpu, MPU6050_PWR_MGMT_1, 0x00);
    if (ret != HAL_OK) {
        return ret;
    }
    HAL_Delay(100);
    ret = MPU6050_WriteRegister(mpu, MPU6050_CONFIG, 0x06);
        if (ret != HAL_OK) {
            return ret;
    }
    HAL_Delay(100);
    ret = MPU6050_WriteRegister(mpu, MPU6050_ACCEL_CONFIG, 0x00);
    if (ret != HAL_OK) {
        return ret;
    }
    ret = MPU6050_WriteRegister(mpu, MPU6050_GYRO_CONFIG, 0x00);
    if (ret != HAL_OK) {
        return ret;
    }
    return HAL_OK;
}
// Reads the accelerometer data from the MPU6050 sensor
HAL_StatusTypeDef MPU6050_ReadAccel(MPU6050_HandleTypeDef *mpu, int16_t* accelData) {
    uint8_t rawData[6];
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(mpu->hi2c, MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, rawData, 6, HAL_MAX_DELAY);
    //HAL_Delay(100);
    if (ret == HAL_OK) {
        accelData[0] = ((int16_t)(rawData[0] << 8 | rawData[1]));
        accelData[1] = ((int16_t)(rawData[2] << 8 | rawData[3]));
        accelData[2] = ((int16_t)(rawData[4] << 8 | rawData[5]));
    }
    return ret;
}
// Reads the gyroscope data from the MPU6050 sensor
HAL_StatusTypeDef MPU6050_ReadGyro(int16_t* gyroData) {
    uint8_t rawData[6];
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(mpuHandle->hi2c, MPU6050_ADDRESS, MPU6050_GYRO_XOUT_H, I2C_MEMADD_SIZE_8BIT, rawData, 6, HAL_MAX_DELAY);
    //HAL_Delay(100);
    if (ret == HAL_OK) {
        gyroData[0] = ((int16_t)(rawData[0] << 8 | rawData[1])) ;
        gyroData[1] = ((int16_t)(rawData[2] << 8 | rawData[3])) ;
        gyroData[2] = ((int16_t)(rawData[4] << 8 | rawData[5])) ;
    }
    gyroData[0] = (float)gyroData[0]/GYRO_SCALE_FACTOR- gx_offset;
    gyroData[1] = (float)gyroData[1]/GYRO_SCALE_FACTOR- gy_offset;
    gyroData[2] = (float)gyroData[2]/GYRO_SCALE_FACTOR- gz_offset;
    return ret;
}
// Writes a value to a register of the MPU6050 sensor
HAL_StatusTypeDef MPU6050_WriteRegister(MPU6050_HandleTypeDef *mpu, uint8_t reg, uint8_t data) {
    return HAL_I2C_Mem_Write(mpu->hi2c, MPU6050_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}
// Reads value of register of the MPU6050 sensor
HAL_StatusTypeDef MPU6050_ReadRegister(MPU6050_HandleTypeDef *mpu, uint8_t reg, uint8_t* data) {
    return HAL_I2C_Mem_Read(mpu->hi2c, MPU6050_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}
// Calibrates the gyroscope offsets of the MPU6050 sensor
void MPU6050_Calibrate() {
    int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
    int16_t gyro[3];
    float pitch_sum = 0, roll_sum = 0;

    for (int i = 0; i < 100; i++) {
        MPU6050_ReadGyro(gyro);
        gx_sum += gyro[0];
        gy_sum += gyro[1];
        gz_sum += gyro[2];
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
        //HAL_Delay(20);
    }
    for (int i = 0; i < 100; i++) {
    		float pitch=0;
    		float roll=0;
    		 MPU6050_CalculatePitchRoll(&pitch,&roll);
    		 pitch_sum+=pitch+pitch_offset;
    		 roll_sum+=roll+roll_offset;
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
            //HAL_Delay(20);
        }
gx_offset = gx_sum / 100;
gy_offset = gy_sum / 100;
gz_offset = gz_sum / 100;

pitch_offset=pitch_sum/100.0;
roll_offset=roll_sum/100.0;
}
void MPU6050_resetPitchRollOffset(){
	pitch_offset=0;
	roll_offset=0;
	gx_offset = 0;
	gy_offset = 0;
	gz_offset = 0;
}
