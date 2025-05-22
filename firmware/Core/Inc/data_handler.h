#ifndef DATA_HANDLER_H
#define DATA_HANDLER_H

#include <stdint.h>
#include <stdbool.h>

#define PACKET_SIZE 97
#define START_BYTE_1 0xAA
#define START_BYTE_2 0x55

typedef enum {
    MODE_IDLE = 0,
	MODE_MANUAL,
	MODE_CALIBRATION,
    MODE_AUTOMATIC,
    MODE_SAFE,
    MODE_COUNT
} OperationMode;

typedef enum {
    MODE_IDLE_CALIB = 0,
	MODE_RESET,
	MODE_SAVE
} CalibrationMode;

typedef enum {
    MODE_PID = 0,
	MODE_PV,
	LQR,
	MODE_CUSTOM2
} ControlMode;
typedef enum {
    MODE_POINT = 0,
	MODE_CIRCLE,
	MODE_INFINITY,
	MODE_PATH
} TrajectoryMode;


typedef enum {
    ERROR_NONE = 0,
    ERROR_CHECKSUM,
    ERROR_INVALID_MODE,
    ERROR_PACKET_SIZE,
    ERROR_COUNT
} ErrorCode;

typedef struct {
    float tolerance;
    float desiredX;
    float desiredY;
    float current_velocity_X;
    float current_velocity_Y;
    float desired_velocity_X;
    float desired_velocity_Y;
    float calibAngleX;
    float calibAngleY;

    OperationMode operationMode;
    CalibrationMode calibrationMode;
    ControlMode controlMode;
    TrajectoryMode trajectoryMode;
    float manualAngleX;
    float manualAngleY;

    float currentX;
    float currentY;

    // PID Parameters
    float PID_KP;
    float PID_KI;
    float PID_KD;

    // PV Parameters
    float PV_KP;
    float PV_KV;
    float ALPHA;
    float trajectroy_scale;
    float trajectroy_speed;
    float trajectroy_base_speed;
    float dt;
    bool ball_detected;
    ErrorCode lastError;
    bool newDataAvailable;
} DataHandler;


void DataHandler_Init(DataHandler* handler);
void DataHandler_ProcessUARTData(DataHandler* handler, uint8_t* data, uint16_t size);
void DataHandler_Reset(DataHandler* handler);
// Getter implementations (existing ones plus new ones)
float DataHandler_GetPID_KP(const DataHandler* handler);
float DataHandler_GetPID_KI(const DataHandler* handler);
float DataHandler_GetPID_KD(const DataHandler* handler);
float DataHandler_GetPV_KP(const DataHandler* handler);
float DataHandler_GetPV_KV(const DataHandler* handler);
CalibrationMode DataHandler_GetCalibrationMode(const DataHandler* handler);
ControlMode DataHandler_GetControlMode(const DataHandler* handler);
float DataHandler_GetTolerance(const DataHandler* handler);
float DataHandler_GetDesiredX(const DataHandler* handler);
float DataHandler_GetDesiredY(const DataHandler* handler);
float DataHandler_GetCalibAngleX(const DataHandler* handler);
float DataHandler_GetCalibAngleY(const DataHandler* handler);
OperationMode DataHandler_GetOperationMode(const DataHandler* handler);
float DataHandler_GetManualAngleX(const DataHandler* handler);
float DataHandler_GetManualAngleY(const DataHandler* handler);
float DataHandler_GetCurrentX(const DataHandler* handler);
float DataHandler_GetCurrentY(const DataHandler* handler);
ErrorCode DataHandler_GetLastError(const DataHandler* handler);
bool DataHandler_IsNewDataAvailable(const DataHandler* handler);
void DataHandler_ClearNewDataFlag(DataHandler* handler);
float DataHandler_GetCurrentVelocityX(const DataHandler* handler);
float DataHandler_GetCurrentVelocityY(const DataHandler* handler);
float DataHandler_GetDesiredVelocityX(const DataHandler* handler);
float DataHandler_GetDesiredVelocityY(const DataHandler* handler);
float DataHandler_GetALPHA(const DataHandler* handler);
TrajectoryMode DataHandler_GetTrajectoryMode(const DataHandler* handler);
void UART_sendDataHandler(DataHandler* data);
#endif // DATA_HANDLER_H
