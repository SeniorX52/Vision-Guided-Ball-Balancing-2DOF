/*
 * servo.h
 *
 *  Created on: Mar 29, 2025
 *      Author: study
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#ifndef SERVO_H
#define SERVO_H

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"
#define SERVO_MIN_ANGLE   	-35.0f
#define SERVO_MAX_ANGLE 	35.0f
#define SERVO_MIN_CALIB_ANGLE   	-90.0f
#define SERVO_MAX_CALIB_ANGLE 		90.0f
#define SERVO_MIN_PULSE  	500      // 0.5ms
#define SERVO_MAX_PULSE 	2500      // 2.5ms


void Servo_Init(void);
void Servo_X_SetAngle(float angle);
void Servo_Y_SetAngle(float angle);
void Servo_X_Calib(float angle);
void Servo_Y_Calib(float angle);
void Servo_X_Save_Calib(void);
void Servo_Y_Save_Calib(void);
void Servo_Reset_Calib(void);
void Flash_Write_Calibration();
void Flash_Read_Calibration();
#endif

#endif /* INC_SERVO_H_ */
