/*
 * control.c
 *
 *  Created on: Apr 1, 2025
 *      Author: study
 */


//#include "control.h"
#include "servo.h"
#include "control.h"
#include "main.h"
#include <stdio.h>
#define FLASH_CONTROL_ADDRESS         0x0800FC00
#define FILTER_COEFF_A1 0.9802f
#define FILTER_COEFF_B1 0.0198f
#define OUTPUT_GAIN 20.0f
#define PWM_MIN 0.5
#define PWM_MAX 2.5
#define N 8
static float KP=0.6f;
static float KI=0.0001f;
static float KD=0.02f;
static float PV_KP=0.75755102f;
static float PV_KV=0.09081633f;
static float ALPHA=0.5;

PIDController PID_X;
PIDController PID_Y;
DataHandler* handler;
float desired_X=0.0f;
float desired_Y=0.0f;
float dt=0.0f;
TrajectoryMode trajectoryMode=MODE_POINT;
float current_X=0.0f;
float current_Y=0.0f;
float tolerance=1.0f;
volatile uint32_t previousTime = 0;
volatile uint32_t currentTime = 0;
char debug_buf2[256];

float current_state[4];
float desired_state[4];
float u[2];
LQR_Controller ctrl;
float K[2][4] = {
    {10.0000, 5.4772, 0.0000, 0.0000},
    {0.0000, 0.0000, 10.0000, 5.4772}
};
float A[4][4] = {
    {0.0, 1.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 1.0},
    {0.0, 0.0, 0.0, 0.0}
};

float B[4][2] = {
    {0.0, 0.0},
    {1.0, 0.0},
    {0.0, 0.0},
    {0.0, 1.0}
};


void Control_init(DataHandler*	dataHandler){
	if(dataHandler!=NULL){
		handler=dataHandler;
	}
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	//Flash_Read_Control_Parameters();
	Control_PID_Init(&PID_X);
	Control_PID_Init(&PID_Y);
	Control_PV_Init();
	LQR_Init(&ctrl);
	currentTime=DWT->CYCCNT;
	UpdateTime();

}
void Control_PID_Init(PIDController* pid) {
    *pid = (PIDController){
        .kp = KP,
        .ki = KI,
        .kd = KD,
        .integral = 0.0f,
        .tolerance = 0.0f,
        .desired_V = 0.0f,
        .current_V = 0.0f,
        .prev_output = 0.0f,
	    .prev_measurement = 0,
	    .prev_output = 0,
	    .prev_time = HAL_GetTick(),
		.delta_pos={0,0,0,0,0,0,0,0},
		.filtered_delta_pos= 0.0f
    };

}

PV_ControllerStates ctrl_x;
PV_ControllerStates ctrl_y;

void Control_PV_Init(void) {
    ctrl_x.prev_pos_filtered = 0.0f;
    ctrl_x.prev_vel_filtered = 0.0f;
    ctrl_x.prev_vel_input = 0.0f;
    ctrl_x.prev_pos = 0.0f;
    ctrl_x.prev_output = 0.0f;

    ctrl_y.prev_pos_filtered = 0.0f;
    ctrl_y.prev_vel_filtered = 0.0f;
    ctrl_y.prev_vel_input = 0.0f;
    ctrl_y.prev_pos = 0.0f;
    ctrl_y.prev_output = 0.0f;
}
void UpdateTime() {
	previousTime = currentTime;
	currentTime = DWT->CYCCNT;
}
float GetDeltaTime() {
    return (currentTime - previousTime) / SystemCoreClock;
}
void LQR_Init(LQR_Controller* ctrl) {
    // Initialize with your gains
    const float K_init[2][4] = {
        {5.6747f, 0.7304f, 0.0f, 0.0f},
        {0.0f, 0.0f, 5.6747f, 0.7304f}
    };
    const float Ki_init[2][2] = {
        {5.0f, 0.0f},
        {0.0f, 5.0f}
    };

    memcpy(ctrl->K, K_init, sizeof(K_init));
    memcpy(ctrl->Ki, Ki_init, sizeof(Ki_init));
    memset(ctrl->xi, 0, sizeof(ctrl->xi));
}
void LQR_Update(LQR_Controller* ctrl, const float x[4], const float r[2], float dt, float u[2]) {
    float y[2] = {x[0], x[2]};
    float e[2];

    e[0] = r[0] - y[0];
    e[1] = r[1] - y[1];

    ctrl->xi[0] += e[0] * dt;
    ctrl->xi[1] += e[1] * dt;

    ctrl->xi[0] = fmaxf(fminf(ctrl->xi[0], 2.0f), -2.0f);
    ctrl->xi[1] = fmaxf(fminf(ctrl->xi[1], 2.0f), -2.0f);

    u[0] = -(ctrl->K[0][0]*x[0] + ctrl->K[0][1]*x[1] +
            ctrl->K[0][2]*x[2] + ctrl->K[0][3]*x[3] +
            ctrl->Ki[0][0]*ctrl->xi[0] + ctrl->Ki[0][1]*ctrl->xi[1]);

    u[1] = -(ctrl->K[1][0]*x[0] + ctrl->K[1][1]*x[1] +
            ctrl->K[1][2]*x[2] + ctrl->K[1][3]*x[3] +
            ctrl->Ki[1][0]*ctrl->xi[0] + ctrl->Ki[1][1]*ctrl->xi[1]);
}
void Control_LQR_UpdateState(float x[4], const float u[2], float dt) {
    float x_dot[4] = {0};

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            x_dot[i] += A[i][j] * x[j];
        }
        for (int j = 0; j < 2; j++) {
            x_dot[i] += B[i][j] * u[j];
        }
    }

    for (int i = 0; i < 4; i++) {
        x[i] += x_dot[i] * dt;
    }
}

void Control_LQR(const float current_state[4], const float desired_state[4], float control_output[2]) {
    float error[4];

    for (int i = 0; i < 4; i++) {
        error[i] = current_state[i] - desired_state[i];
    }

    control_output[0] = -(K[0][0]*error[0] + K[0][1]*error[1] +
                        K[0][2]*error[2] + K[0][3]*error[3]);

    control_output[1] = -(K[1][0]*error[0] + K[1][1]*error[1] +
                        K[1][2]*error[2] + K[1][3]*error[3]);
}

bool newsetpoint=false;
#include <math.h>
void Control_PID2(PIDController* pid, float setpoint, float measurement, float* output) {
    float dt = 0.01f; // to be measured later after FTDI

    // === position prediction ===
//    float theta = (pid->prev_output / 180.0f) * 3.14f;
//    float accel = 2.047 * sin(theta);
//    pid->current_V = pid->current_V + accel * dt;
//    float predicted_position = measurement + pid->current_V * dt + 0.5f * accel * dt * dt;
    float error = setpoint - measurement;
    float abs_error = fabsf(error);
    // === adaptive gain ===
    float error_normalized = fminf(abs_error / 13.5f, 1.0f);
    float velocity_normalized = fminf(fabsf(pid->current_V) / 40.0f, 1.0f);

    float adaptive_kp = pid->kp * (0.3f + 0.7f * error_normalized);
    float adaptive_kd = pid->kd * (0.5f + 0.5f * velocity_normalized);
    float adaptive_ki = pid->ki * (0.3f + 0.7f * error_normalized);
    bool within_tolerance = (abs_error <= pid->tolerance);
    if (within_tolerance) {
        adaptive_kp *= 0.1f;
        adaptive_kd *= 0.5f;
        adaptive_ki = 0.0f;
        float prop = adaptive_kp * error;
        float derivative = -adaptive_kd * pid->current_V;
        pid->derivative=derivative;
        pid->prop=prop;
        pid->integral *= 0.99f;
        float output_raw = prop + pid->integral + derivative;
        float filter_alpha = 0.5f;
        *output = filter_alpha * pid->prev_output + (1.0f - filter_alpha) * output_raw;
        pid->prev_output = *output;
        return;
    }

    // === increase damping ===
    if (copysignf(1.0f, pid->current_V) != copysignf(1.0f, error)) {
        float damping_boost = 2.5f + 4.0f * expf(-fabsf(error) /0.03);
        adaptive_kd *= damping_boost;
    }

    // === braking Zone ===
    float brake_zone = 2.0f + 3.0f * (1.0f - expf(-fabsf(pid->current_V)/5.0f));
    if (fabsf(measurement - setpoint) < brake_zone &&
       fabsf(pid->current_V) > (5.0f*handler->trajectroy_speed)) {
        adaptive_kd *= 1.5f + velocity_normalized;
    }

    // === intergral I ===
    if((pid->current_V>1.0f) || abs_error>(10.0f*handler->trajectroy_scale)){
    	pid->integral += adaptive_ki * error * dt;
    }
    pid->integral = fmaxf(fminf(pid->integral, 20.0f), -20.0f);

    // === pid output ===
    float prop = adaptive_kp * error;
    float derivative = -adaptive_kd * pid->current_V;
    float output_raw = prop + pid->integral + derivative;

    *output = ALPHA * pid->prev_output + (1.0f - ALPHA) * output_raw;
    pid->prev_output = *output;
}
void Control_PID(PIDController* pid,float setpoint, float measurement, float* output){
	//UpdateTime();
	//dt = GetDeltaTime();
	if(handler->ball_detected ==0){
		pid->derivative=0;
		pid->integral=0;
		pid->prop=0;
		*output=0;
		return;
	}
	dt=handler->dt;
	float error= setpoint-measurement;

	float prop=pid->kp*error;
	pid->prop=prop;
	float derivative=(pid->kd) * (-(pid->current_V));
	if((newsetpoint && trajectoryMode==MODE_POINT) ){
		pid->integral=0;

	}
	if ((fabs(error) < tolerance)) {
//			pid->integral = 0.0f;
			derivative*=0.5;
//			pid->prop=0;
//			*output=0;
//			return;
		}
		else{
			pid->integral+=(pid->ki)*(error)*dt;
		}
    float integral_limit = 23.5f;
    if (pid->integral > integral_limit) pid->integral = integral_limit;
    if (pid->integral < -integral_limit) pid->integral = -integral_limit;
	pid->derivative=derivative;


	pid->prev_error=error;

	float output_raw=prop+pid->integral+derivative;
	// EMA
	*output = ALPHA * pid->prev_output + (1 - ALPHA) * output_raw;
	pid->prev_output=*output;
	pid->derivative=ALPHA * pid->prev_derivative + (1 - ALPHA) * pid->derivative;
}


void Control_PV(PV_ControllerStates* ctrl, float setpoint, float measurement, float* output) {
//	float pos_filtered = 0.3333f * (measurement + ctrl->prev_pos) + 0.3333f * ctrl->prev_pos_filtered;
//    float vel_filtered=0.3333f*(ctrl->prev_vel_filtered)+66.67f*PV_KV*pos_filtered-66.67f*ctrl->prev_vel_input;
//    ctrl->prev_vel_input=PV_KV*pos_filtered;
//    ctrl->prev_vel_filtered=vel_filtered;
//    ctrl->prev_pos_filtered=pos_filtered;
//    ctrl->prev_pos=measurement;
//    float error = PV_KP*(setpoint - pos_filtered);
//    *output = error - vel_filtered;
//    ctrl->prev_output = *output;
	float error = PV_KP*(setpoint - measurement);
	*output = error - PV_KV*(ctrl->current_V);
    *output = ALPHA * ctrl->prev_output + (1.0f - ALPHA) * (*output);
    ctrl->prev_output = *output;

    /*
    char debug_buf[64];
    snprintf(debug_buf, sizeof(debug_buf), "PV_Output=%.2f\r\n", *output);
    UART_sendString(debug_buf);
    */
}

LeadFilter myLead_X = {
    .b0 = 46.2f,
    .b1 = -41.8f,
    .a1 = 0.6f,
    .x_prev = 0,
    .y_prev = 0
};
LeadFilter myLead_Y = {
    .b0 = 46.2f,
    .b1 = -41.8f,
    .a1 = 0.6f,
    .x_prev = 0,
    .y_prev = 0
};
void Control_Loop(){
	ControlMode controlMode=handler->controlMode;
	UpdateTime();
	float servo_X_output=0;
	float servo_Y_output=0;
	Control_update_parameters();
	switch(controlMode){
		case MODE_PID:
		{
			Control_PID(&PID_X, desired_X, current_X,&servo_X_output);
			Control_PID(&PID_Y, desired_Y, current_Y,&servo_Y_output);

			break;
		}
		case MODE_PV:
		{
			Control_PID2(&PID_X, desired_X, current_X,&servo_X_output);
			Control_PID2(&PID_Y, desired_Y, current_Y,&servo_Y_output);

			break;
		}
		case LQR:
		{
            const float dt = handler->dt;

            Control_LQR(current_state, desired_state, u);

            Control_LQR_UpdateState(current_state, u, dt);

            servo_X_output = u[0];
            servo_Y_output = u[1];
            break;
		}
		case MODE_CUSTOM2:
		{

			LeadCompensator_Update(&myLead_X,desired_X, current_X,&servo_X_output);
			LeadCompensator_Update(&myLead_Y,desired_Y, current_Y,&servo_Y_output);
			break;
		}
		default:
		     break;
	}
	Servo_X_SetAngle(servo_X_output);
	Servo_Y_SetAngle(servo_Y_output);
	snprintf(debug_buf2, sizeof(debug_buf2),
	    "{\"Servo_X\":%.2f,\"Servo_Y\":%.2f,\"desired_X\":%.2f,\"desired_Y\":%.2f,"
	    "\"kp_x\":%.2f,\"kd_x\":%.2f,\"ki_x\":%.2f,"
	    "\"kp_y\":%.2f,\"kd_y\":%.2f,\"ki_y\":%.2f}",
		servo_X_output, servo_Y_output,
	    desired_X, desired_Y,
	    PID_X.prop, PID_X.derivative, PID_X.integral,
	    PID_Y.prop, PID_Y.derivative, PID_Y.integral);

	UART_sendString(debug_buf2);
	UART_sendString("\r\n");
	memset(debug_buf2, 0, sizeof(debug_buf2));
	//HAL_Delay(17);

}
void LeadCompensator_Update(LeadFilter* comp, float setpoint, float measurement, float* output) {
	float input=setpoint - measurement;
    *output = comp->b0 * input + comp->b1 * comp->x_prev - comp->a1 * comp->y_prev;
    *output = ALPHA * comp->y_prev + (1.0f - ALPHA) * (*output);
    comp->x_prev = input;
    comp->y_prev = *output;
}
float prev_setpoint_x=0.0f;
float prev_setpoint_y=0.0f;

void Control_update_parameters(){
	if(desired_X!=prev_setpoint_x || desired_Y!=prev_setpoint_y){
		newsetpoint=true;
	}
	else{
		newsetpoint=false;
	}
	prev_setpoint_x=desired_X;
	prev_setpoint_y=desired_Y;
	trajectoryMode = handler->trajectoryMode;
	if (trajectoryMode == MODE_POINT || trajectoryMode == MODE_PATH) {
		desired_X = handler->desiredX;
		desired_Y = handler->desiredY;
		PID_X.desired_V = 0;
		PID_Y.desired_V = 0;
		ctrl_x.desired_V = 0;
		ctrl_y.desired_V = 0;
	}

	tolerance = handler->tolerance;
	current_X = handler->currentX;
	current_Y = handler->currentY;

	if(handler->controlMode==MODE_PID){
		PID_X.kp = KP;
		PID_X.ki = KI;
		PID_X.kd = KD;
		PID_Y.kp = PV_KP;
		PID_Y.ki = KI;
		PID_Y.kd = PV_KV;
		PV_KP = (handler->PV_KP < 0.0f) ? PV_KP : handler->PV_KP;
		PV_KV = (handler->PV_KV < 0.0f) ? PV_KV : handler->PV_KV;
		KP = (handler->PID_KP < 0.0f) ? KP : handler->PID_KP;
		KI = (handler->PID_KI < 0.0f) ? KI : handler->PID_KI;
		KD = (handler->PID_KD < 0.0f) ? KD : handler->PID_KD;
		PID_X.current_V = handler->current_velocity_X;
		PID_Y.current_V = handler->current_velocity_Y;
	}
	if(handler->controlMode==MODE_PV){
		PV_KP = (handler->PV_KP < 0.0f) ? PV_KP : handler->PV_KP;
		PV_KV = (handler->PV_KV < 0.0f) ? PV_KV : handler->PV_KV;
		ctrl_x.current_V = handler->current_velocity_X;
		ctrl_y.current_V = handler->current_velocity_Y;
		ctrl_x.desired_V = handler->desired_velocity_X;
		ctrl_y.desired_V = handler->desired_velocity_Y;
	}

	if(handler->controlMode==LQR){
		current_state[0] = current_X;
		current_state[1] = handler->current_velocity_X;
		current_state[2] = current_Y;
		current_state[3] = handler->current_velocity_Y;

		desired_state[0] = desired_X;
		desired_state[1] = 0;
		desired_state[2] = desired_Y;
		desired_state[3] = 0;
	}
	ALPHA = handler->ALPHA;
	// Flash_Write_Control_Parameters();
}

static uint32_t last_update_time = 0;
static float angle_rad = 0.0f;

void generate_circle_points() {
    uint32_t current_time = HAL_GetTick();
    uint32_t elapsed_time = current_time - last_update_time;

    if (elapsed_time >= (10 * (1.0f - handler->trajectroy_speed))) {
        last_update_time = current_time;

        float radius = handler->trajectroy_scale * 12;
        float angular_speed = handler->trajectroy_base_speed * handler->trajectroy_speed;

        angle_rad += angular_speed * (elapsed_time / 1000.0f);
        if (angle_rad > 2 * 3.14f) {
            angle_rad -= 2 * 3.14f;
        }

        desired_X = radius * cosf(angle_rad);
        desired_Y = radius * sinf(angle_rad);
        Control_Loop();
    }
}

void generate_infinity_points() {
    uint32_t current_time = HAL_GetTick();
    uint32_t elapsed_time = current_time - last_update_time;

    if (elapsed_time >= (10 * (1.0f - handler->trajectroy_speed))) {
        last_update_time = current_time;

        float radius = handler->trajectroy_scale * 12;
        float angular_speed = handler->trajectroy_base_speed * handler->trajectroy_speed;

        angle_rad += angular_speed * (elapsed_time / 1000.0f);
        if (angle_rad > 2 * 3.14f) {
            angle_rad -= 2 * 3.14f;
        }

        desired_X = radius * sinf(angle_rad);
        desired_Y = radius * sinf(angle_rad) * cosf(angle_rad);
        Control_Loop();
    }
}

void Control_Reset(){
    float kp = PID_X.kp;
    float ki = PID_X.ki;
    float kd = PID_X.kd;
    PID_X = (PIDController){
        .kp = kp, .ki = ki, .kd = kd,
        .integral = 0.0f,
        .tolerance = 0.0f,
        .desired_V = 0.0f,
        .current_V = 0.0f,
        .prev_output = 0.0f
    };

    PID_Y = (PIDController){
            .kp = kp, .ki = ki, .kd = kd,
            .integral = 0.0f,
            .tolerance = 0.0f,
            .desired_V = 0.0f,
            .current_V = 0.0f,
            .prev_output = 0.0f
        };
    Control_PV_Init();
    LQR_Init(&ctrl);
}



void Flash_Write_Control_Parameters() {
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = FLASH_CONTROL_ADDRESS;
    EraseInitStruct.NbPages = 1;
    HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_CONTROL_ADDRESS,*(uint32_t*)&KP);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_CONTROL_ADDRESS + 4,  *(uint32_t*)&KI);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_CONTROL_ADDRESS + 8,  *(uint32_t*)&KD);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_CONTROL_ADDRESS + 12, *(uint32_t*)&PV_KP);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_CONTROL_ADDRESS + 16, *(uint32_t*)&PV_KV);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_CONTROL_ADDRESS + 20, *(uint32_t*)&ALPHA);
    HAL_FLASH_Lock();
}

void Flash_Read_Control_Parameters() {
    KP = Read_Float_From_Flash(FLASH_CONTROL_ADDRESS,KP);
    KI = Read_Float_From_Flash(FLASH_CONTROL_ADDRESS + 4,KI);
    KD = Read_Float_From_Flash(FLASH_CONTROL_ADDRESS + 8,KD);

    PV_KP = Read_Float_From_Flash(FLASH_CONTROL_ADDRESS + 12,PV_KP);
    PV_KV = Read_Float_From_Flash(FLASH_CONTROL_ADDRESS + 16, PV_KV);
    ALPHA = Read_Float_From_Flash(FLASH_CONTROL_ADDRESS + 20, ALPHA);
}

float Read_Float_From_Flash(uint32_t address, float default_val) {
    uint32_t val = *(uint32_t*)address;
    return (val == 0xFFFFFFFF) ? default_val : *(float*)&val;
}


