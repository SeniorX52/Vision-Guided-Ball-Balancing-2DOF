/*
 * data_handler.c
 *
 *  Created on: Mar 30, 2025
 *      Author: study
 */

#include "data_handler.h"
#include <string.h>
#include "main.h"


static void parseDataPacket(DataHandler* handler, uint8_t* data);
static void handleError(DataHandler* handler, ErrorCode error);

void DataHandler_Init(DataHandler* handler) {
    if (handler == NULL) return;
    memset(handler, 0, sizeof(DataHandler));
    handler->operationMode = MODE_IDLE;
    handler->calibrationMode = MODE_IDLE;
    handler->controlMode = MODE_PID;
    handler->lastError = ERROR_NONE;
}

void DataHandler_ProcessUARTData(DataHandler* handler, uint8_t* data, uint16_t size) {
    if (handler == NULL || data == NULL || size != PACKET_SIZE) {
        handleError(handler, ERROR_PACKET_SIZE);
        return;
    }
    parseDataPacket(handler, data);
    handler->newDataAvailable = true;
}

static void parseDataPacket(DataHandler* handler, uint8_t* data) {
    uint8_t* ptr = data;

    // control Parameters
    memcpy(&handler->tolerance, ptr, sizeof(float)); ptr += sizeof(float);
    memcpy(&handler->desiredX, ptr, sizeof(float)); ptr += sizeof(float);
    memcpy(&handler->desiredY, ptr, sizeof(float)); ptr += sizeof(float);

    //velocity
    memcpy(&handler->current_velocity_X, ptr, sizeof(float)); ptr += sizeof(float);
    memcpy(&handler->current_velocity_Y, ptr, sizeof(float)); ptr += sizeof(float);
    memcpy(&handler->desired_velocity_X, ptr, sizeof(float)); ptr += sizeof(float);
    memcpy(&handler->desired_velocity_Y, ptr, sizeof(float)); ptr += sizeof(float);
    // calibration Data
    memcpy(&handler->calibAngleX, ptr, sizeof(float)); ptr += sizeof(float);
    memcpy(&handler->calibAngleY, ptr, sizeof(float)); ptr += sizeof(float);

    // operation Modes
    uint8_t opMode = *ptr++;
    uint8_t calMode = *ptr++;
    uint8_t ctrlMode = *ptr++;
    uint8_t trajMode = *ptr++;

    // validate modes
    handler->operationMode = opMode;
    handler->calibrationMode =calMode;
    handler->controlMode =ctrlMode;
    handler->trajectoryMode =trajMode;


    // manual Control
    memcpy(&handler->manualAngleX, ptr, sizeof(float)); ptr += sizeof(float);
    memcpy(&handler->manualAngleY, ptr, sizeof(float)); ptr += sizeof(float);

    // system State
    memcpy(&handler->currentX, ptr, sizeof(float)); ptr += sizeof(float);
    memcpy(&handler->currentY, ptr, sizeof(float)); ptr += sizeof(float);

    // pid parameters
    memcpy(&handler->PID_KP, ptr, sizeof(float)); ptr += sizeof(float);
    memcpy(&handler->PID_KI, ptr, sizeof(float)); ptr += sizeof(float);
    memcpy(&handler->PID_KD, ptr, sizeof(float)); ptr += sizeof(float);

    // pv parameters
    memcpy(&handler->PV_KP, ptr, sizeof(float)); ptr += sizeof(float);
    memcpy(&handler->PV_KV, ptr, sizeof(float)); ptr += sizeof(float);
    memcpy(&handler->ALPHA, ptr, sizeof(float)); ptr += sizeof(float);

    memcpy(&handler->trajectroy_scale, ptr, sizeof(float)); ptr += sizeof(float);
    memcpy(&handler->trajectroy_speed, ptr, sizeof(float)); ptr += sizeof(float);
    memcpy(&handler->trajectroy_base_speed, ptr, sizeof(float));ptr += sizeof(float);
    memcpy(&handler->dt, ptr, sizeof(float));ptr += sizeof(float);
    handler->ball_detected = *ptr;
}

void DataHandler_Reset(DataHandler* handler) {
    if (handler == NULL) return;

    handler->tolerance = 0.0f;
    handler->desiredX = 0.0f;
    handler->desiredY = 0.0f;

    handler->current_velocity_X= 0.0f;
    handler->current_velocity_Y = 0.0f;

    handler->desired_velocity_X= 0.0f;
    handler->desired_velocity_Y = 0.0f;

    handler->calibAngleX = 0.0f;
    handler->calibAngleY = 0.0f;


    handler->operationMode = MODE_IDLE;
    handler->calibrationMode = MODE_IDLE;
    handler->controlMode = MODE_PID;
    handler->trajectoryMode = MODE_POINT;


    handler->manualAngleX = 0.0f;
    handler->manualAngleY = 0.0f;


    handler->currentX = 0.0f;
    handler->currentY = 0.0f;

    handler->PID_KP = 0.0f;
    handler->PID_KI = 0.0f;
    handler->PID_KD = 0.0f;
    handler->PV_KP = 0.0f;
    handler->PV_KV = 0.0f;
    handler->ALPHA = 0.0f;
    handler->dt = 0.0f;
    // Status
    handler->lastError = ERROR_NONE;
    handler->newDataAvailable = false;
}
void UART_sendDataHandler(DataHandler* data) {
    char buffer[512];

    int len = snprintf(buffer, sizeof(buffer),
        "tolerance=%.2f\n,desiredX=%.2f\n,desiredY=%.2f,\n"
        "current_velocity_X=%.2f\n,current_velocity_Y=%.2f,\n"
        "desired_velocity_X=%.2f\n,desired_velocity_Y=%.2f\n,"
        "calibAngleX=%.2f\n,calibAngleY=%.2f\n,"
        "operationMode=%d\n,calibrationMode=%d\n,controlMode=%d,\n"
        "manualAngleX=%.2f\n,manualAngleY=%.2f\n,"
        "currentX=%.2f\n,currentY=%.2f,\n"
        "PID_KP=%.2f\n,PID_KI=%.2f\n,PID_KD=%.2f,\n"
        "PV_KP=%.2f\n,PV_KV=%.2f\n,"
        "lastError=%d\n,newDataAvailable=%d\n\n",
        data->tolerance,
        data->desiredX,
        data->desiredY,
        data->current_velocity_X,
        data->current_velocity_Y,
        data->desired_velocity_X,
        data->desired_velocity_Y,
        data->calibAngleX,
        data->calibAngleY,
        (int)data->operationMode,
        (int)data->calibrationMode,
        (int)data->controlMode,
        data->manualAngleX,
        data->manualAngleY,
        data->currentX,
        data->currentY,
        data->PID_KP,
        data->PID_KI,
        data->PID_KD,
        data->PV_KP,
        data->PV_KV,
        (int)data->lastError,
        data->newDataAvailable ? 1 : 0
    );

    if (len > 0 && len < sizeof(buffer)) {
        UART_sendString(buffer);
    } else {
        UART_sendString("[ERROR] Data overflow\n");
    }
}

static void handleError(DataHandler* handler, ErrorCode error) {
    if (handler == NULL) return;
    handler->lastError = error;
}

// Getter implementations (existing ones plus new ones)
float DataHandler_GetPID_KP(const DataHandler* handler) {
    return handler != NULL ? handler->PID_KP : 0.0f;
}

float DataHandler_GetPID_KI(const DataHandler* handler) {
    return handler != NULL ? handler->PID_KI : 0.0f;
}

float DataHandler_GetPID_KD(const DataHandler* handler) {
    return handler != NULL ? handler->PID_KD : 0.0f;
}

float DataHandler_GetPV_KP(const DataHandler* handler) {
    return handler != NULL ? handler->PV_KP : 0.0f;
}

float DataHandler_GetPV_KV(const DataHandler* handler) {
    return handler != NULL ? handler->PV_KV : 0.0f;
}

CalibrationMode DataHandler_GetCalibrationMode(const DataHandler* handler) {
    return handler != NULL ? handler->calibrationMode : MODE_IDLE;
}

ControlMode DataHandler_GetControlMode(const DataHandler* handler) {
    return handler != NULL ? handler->controlMode : MODE_PID;
}

float DataHandler_GetTolerance(const DataHandler* handler) {
    return handler != NULL ? handler->tolerance : 0.0f;
}

float DataHandler_GetDesiredX(const DataHandler* handler) {
    return handler != NULL ? handler->desiredX : 0.0f;
}

float DataHandler_GetDesiredY(const DataHandler* handler) {
    return handler != NULL ? handler->desiredY : 0.0f;
}
float DataHandler_GetCurrentVelocityX(const DataHandler* handler) {
    return handler != NULL ? handler->current_velocity_X : 0.0f;
}

float DataHandler_GetCurrentVelocityY(const DataHandler* handler) {
    return handler != NULL ? handler->current_velocity_Y : 0.0f;
}

float DataHandler_GetCalibAngleX(const DataHandler* handler) {
    return handler != NULL ? handler->calibAngleX : 0.0f;
}

float DataHandler_GetCalibAngleY(const DataHandler* handler) {
    return handler != NULL ? handler->calibAngleY : 0.0f;
}

OperationMode DataHandler_GetOperationMode(const DataHandler* handler) {
    return handler != NULL ? handler->operationMode : MODE_IDLE;
}

float DataHandler_GetManualAngleX(const DataHandler* handler) {
    return handler != NULL ? handler->manualAngleX : 0.0f;
}

float DataHandler_GetManualAngleY(const DataHandler* handler) {
    return handler != NULL ? handler->manualAngleY : 0.0f;
}

float DataHandler_GetCurrentX(const DataHandler* handler) {
    return handler != NULL ? handler->currentX : 0.0f;
}

float DataHandler_GetCurrentY(const DataHandler* handler) {
    return handler != NULL ? handler->currentY : 0.0f;
}

ErrorCode DataHandler_GetLastError(const DataHandler* handler) {
    return handler != NULL ? handler->lastError : ERROR_NONE;
}

bool DataHandler_IsNewDataAvailable(const DataHandler* handler) {
    return handler != NULL ? handler->newDataAvailable : false;
}

void DataHandler_ClearNewDataFlag(DataHandler* handler) {
    if (handler != NULL) {
        handler->newDataAvailable = false;
    }
}
float DataHandler_GetDesiredVelocityX(const DataHandler* handler){
	return handler != NULL ? handler->desired_velocity_X : false;
}
float DataHandler_GetDesiredVelocityY(const DataHandler* handler){
	return handler != NULL ? handler->desired_velocity_Y : false;
}
float DataHandler_GetALPHA(const DataHandler* handler){
	return handler != NULL ? handler->ALPHA: false;
}
TrajectoryMode DataHandler_GetTrajectoryMode(const DataHandler* handler){
	return handler != NULL ? handler->trajectoryMode : false;
}

