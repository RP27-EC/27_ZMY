#ifndef __DM_MOTOR_H
#define __DM_MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"
#include "arm_math.h"
#include "drv_can.h"
#include "drv_tick.h"
#include "rp_math.h"
/* Exported typedef ----------------------------------------------------------*/
typedef enum Motor_MIT_Command_enum_e
{
	Enter_Motor_Mode,
	Clear_Err_Code,

}Motor_MIT_Command_e;

#define P_MIN -12.5f   
#define P_MAX 12.5f
#define P_CYCLE (P_MAX - P_MIN)
#define P_HALF  ((P_MAX - P_MIN) / 2.f)
#define V_MIN -30.0f    
#define V_MAX 30.0f
#define KP_MIN 0.0f    
#define KP_MAX 500.0f
#define KD_MIN 0.0f     
#define KD_MAX 5.0f
#define T_MIN -10.0f    
#define T_MAX 10.0f


#define DM_MOTOR_LOST_TICK 50

typedef enum Motor_DM_State_enum_e
{
	DM_Disable       = 0,
	DM_Enable        = 1,
	DM_OverVoltage   = 8,
	DM_LackVoltage   = 9,
	DM_OverCurrent   = 10,
	DM_MosOverTemp   = 11,
	DM_MotorOverTemp = 12,
	DM_CommunLoss    = 13,
}Motor_DM_State_e;

typedef struct Motor_DM_Born_Data_struct_t
{
	uint32_t tx_id;

	CAN_HandleTypeDef *hcan;


}Motor_DM_Born_Data_t;

typedef struct Motor_DM_Rx_Data_struct_t
{
	float speed;

	float torque;

	float motor_angle;

	float motor_angle_sum;

	float motor_angle_last;

	uint8_t angle_sum_flag;

	uint32_t last_rx_tick;
}Motor_DM_Rx_Data_t;

typedef struct Motor_DM_Tx_Data_struct_t
{
	float torque;

	float target_speed;

	float target_angle;

	float Kp;

	float Kd;

	uint8_t tx_buff[8];
}Motor_DM_Tx_Data_t;

typedef struct Motor_DM_State_Data_struct_t
{
	Motor_DM_State_e  err_code;
}Motor_DM_State_Data_t;

typedef struct Motor_DM_struct_t
{
	Motor_DM_Born_Data_t* born_data;

	Motor_DM_Rx_Data_t* rx_data;

	Motor_DM_Tx_Data_t* tx_data;

	Motor_DM_State_Data_t* state_data;
}Motor_DM_t;

uint8_t DM_Motor_Is_Ready(Motor_DM_t *motor);
void DM_Motor_Coast(Motor_DM_t *motor);
void DM_Motor_Update(Motor_DM_t *motor, uint8_t *rxBuf);
void DM_Motor_Sleep(Motor_DM_t *motor);
void DM_Motor_ZeroPos(Motor_DM_t *motor);
void DM_Set_Torque(Motor_DM_t *motor);

#endif
