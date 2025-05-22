/*
 * control.h
 *
 *  Created on: Apr 1, 2025
 *      Author: study
 */

#include "data_handler.h"
#include "stm32f1xx_hal.h"
#include "math.h"

typedef struct{
	float kp,ki,kd;
	float integral;
	float derivative;
	float prev_derivative;
	float prop;
	float tolerance;
	float desired_V;
	float current_V;
	float prev_output;
	float prev_velocity;
	float prev_error;
	float prev_measurement;
	float delta_pos[8];
	float filtered_delta_pos;
    uint32_t prev_time;

} PIDController;

typedef struct {
    float prev_pos_filtered;
    float prev_vel_filtered;
    float prev_vel_input;
    float prev_pos;
	float desired_V;
	float current_V;
    float prev_output;
} PV_ControllerStates;
typedef struct {
    float b0;
    float b1;
    float a1;
    float x_prev;
    float y_prev;
} LeadFilter;
typedef struct {
    float xi[2];
    float K[2][4];
    float Ki[2][2];
} LQR_Controller;

// -------------------MPC Configuration----------------------------
#define PREDICTION_HORIZON 10
#define CONTROL_HORIZON 2
#define SAMPLE_TIME_MS 22

// System dimensions
#define NX 3  // Number of states
#define NU 1  // Number of inputs
#define NY 1  // Number of outputs

// Constraints
#define U_MIN -1.0f
#define U_MAX 1.0f
#define U_RATE_MIN -0.1f
#define U_RATE_MAX 0.1f

typedef struct {
    float x[NX];       // State vector
    float u_prev;      // Previous control input
    float y_ref;       // Current reference
} MPCState;

void mpc_init(MPCState* mpc);
float mpc_compute(MPCState* mpc, float y_measured);
// ----------------------------------------------------------------------

void Control_init(DataHandler*	dataHandler);
void UpdateTime();
float GetDeltaTime();
void Control_Loop();
void Control_PID(PIDController* pid,float setpoint, float measurement, float* output);
void Control_update_parameters();
void Control_Reset();
float compute_tolerance_X();
float compute_tolerance_Y();
float Read_Float_From_Flash(uint32_t address, float default_val);
void Flash_Read_Control_Parameters();
void Flash_Write_Control_Parameters();
void Control_PID_Init(PIDController* pid);
void Control_PV_Init(void);
void Control_PV(PV_ControllerStates* ctrl,float setpoint, float measurement,float* output);
void Control_LQR_UpdateState(float x[4], const float u[2], float dt);
void Control_LQR(const float current_state[4],const float desired_state[4],float control_output[2]);
void generate_circle_points();
void generate_infinity_points();
void LeadCompensator_Update(LeadFilter* comp, float setpoint, float measurement, float* output);
void LQR_Update(LQR_Controller* ctrl, const float x[4], const float r[2], float dt, float u[2]);
void LQR_Init(LQR_Controller* ctrl);
void Control_PID2(PIDController* pid, float setpoint, float measurement, float* output);
