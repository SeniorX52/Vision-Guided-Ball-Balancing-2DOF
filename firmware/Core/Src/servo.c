#include "servo.h"
#include "main.h"
#include "MPU6050.h"
#define FLASH_CALIBRATION_ADDRESS  0x0800F800



TIM_HandleTypeDef htim2;
char debug_buf[64];
static float X_REF=-12.0f;
static float Y_REF=-21.4f;
static float current_x=0;
static float current_y=0;
static float pitch=0;
static float roll=0;
void Servo_Init(void) {
    TIM_OC_InitTypeDef sConfigOC = {0};
    __HAL_RCC_TIM2_CLK_ENABLE();
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 63;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 19999;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim2);
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 1500;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    X_REF=5.4f;
    Y_REF=-5.4f;
    //Flash_Read_Calibration();
    Servo_X_SetAngle(0);
    Servo_Y_SetAngle(0);

}



void Servo_X_SetAngle(float angle) {
	if (angle < SERVO_MIN_ANGLE) angle =SERVO_MIN_ANGLE ;
	if (angle > SERVO_MAX_ANGLE ) angle = SERVO_MAX_ANGLE ;
	//MPU6050_CalculatePitchRoll(&pitch, &roll);
    angle+=X_REF+90+pitch;
    uint32_t pulse = 500 + (angle / 180.0) * 2000;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);

}

void Servo_Y_SetAngle(float angle) {
	if (angle < SERVO_MIN_ANGLE) angle =SERVO_MIN_ANGLE ;
	if (angle > SERVO_MAX_ANGLE ) angle = SERVO_MAX_ANGLE ;
	//MPU6050_CalculatePitchRoll(&pitch, &roll);
    angle+=Y_REF+90+roll;
    uint32_t pulse = 500 + (angle / 180.0) * 2000;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse);
}

void Servo_X_Calib(float angle) {
    if (angle < SERVO_MIN_CALIB_ANGLE) angle =SERVO_MIN_CALIB_ANGLE ;
    if (angle > SERVO_MAX_CALIB_ANGLE ) angle = SERVO_MAX_CALIB_ANGLE ;
    current_x=angle+X_REF;
    angle=current_x+90;
    uint32_t pulse = 500 + (angle / 180.0) * 2000;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);
//    sprintf(debug_buf, "XC:%.2f,P:%lu\r\n", angle-90, pulse);
//    UART_sendString(debug_buf);
}

void Servo_Y_Calib(float angle) {
	if (angle < SERVO_MIN_CALIB_ANGLE) angle =SERVO_MIN_CALIB_ANGLE ;
	if (angle > SERVO_MAX_CALIB_ANGLE ) angle = SERVO_MAX_CALIB_ANGLE ;
    current_y=angle+Y_REF;
    angle=current_y+90;
    uint32_t pulse = 500 + (angle / 180.0) * 2000;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse);
//    sprintf(debug_buf, "YC:%.2f,P:%lu\r\n", angle-90, pulse);
//    UART_sendString(debug_buf);
}


void Servo_X_Save_Calib(void){
    X_REF=current_x;
    //Flash_Write_Calibration();
}

void Servo_Y_Save_Calib(void){
    Y_REF=current_y;
    //Flash_Write_Calibration();

}

void Servo_Reset_Calib(void){
    X_REF=0;
    Y_REF=0;
    current_x=0;
    current_y=0;
    //Flash_Write_Calibration();
}
void Flash_Write_Calibration() {
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = FLASH_CALIBRATION_ADDRESS;
    EraseInitStruct.NbPages = 1;
    HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_CALIBRATION_ADDRESS, *(uint32_t*)&X_REF);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_CALIBRATION_ADDRESS + 4, *(uint32_t*)&Y_REF);
    HAL_FLASH_Lock();
}
void Flash_Read_Calibration() {
    uint32_t storedX = *(uint32_t*)FLASH_CALIBRATION_ADDRESS;
    uint32_t storedY = *(uint32_t*)(FLASH_CALIBRATION_ADDRESS + 4);
    if (storedX == 0xFFFFFFFF || storedY == 0xFFFFFFFF) {
        X_REF = 0.0f;
        Y_REF = 0.0f;
    } else {
        X_REF = *(float*)&storedX;
        Y_REF = *(float*)&storedY;
    }
}

