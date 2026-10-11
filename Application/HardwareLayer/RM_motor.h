#ifndef __RM_MOTOR_H
#define __RM_MOTOR_H

#include "rp_config.h"
#include "pid.h"
#include "drv_can.h"


#define _3508_TORQUE_CONSTANT     0.3f
#define _3508_MAX_CURRENT         20.f
#define _3508_REDUCT_RATIO        (19.f/1.f)

typedef struct Motor_RM_Born_Data_struct_t
{
	uint8_t rxId;

	uint32_t stdId;

	FDCAN_HandleTypeDef *hcan;
}Motor_RM_Born_Data_t;

typedef struct Motor_RM_Rx_Data_struct_t
{
	int16_t encoder_speed;

	float speed;

	uint16_t encoder;

	int16_t torque_current_raw;

	int8_t temperature;

	uint32_t last_rx_tick;
}Motor_RM_Rx_Data_t;

typedef struct Motor_RM_Ctrl_Data_struct_t
{
	pid_ctrl_t* speed_ctrl;
}Motor_RM_Ctrl_Data_t;

typedef struct Motor_RM_Tx_Data_struct_t
{
	float torque;

	float torque_current;

	int16_t torque_current_raw;
}Motor_RM_Tx_Data_t;

typedef struct Motor_RM_struct_t
{
	Motor_RM_Born_Data_t* born_data;

	Motor_RM_Rx_Data_t* rx_data;

	Motor_RM_Tx_Data_t* tx_data;

	Motor_RM_Ctrl_Data_t* ctrl_data;
}Motor_RM_t;

typedef struct Motor_RM_Set_struct_t
{
	Motor_RM_t* motor[4];

	uint8_t tx_buff[8];

	uint32_t stdId;

	FDCAN_HandleTypeDef *hcan;
}Motor_RM_Set_t;


void RM_Motor_Update(Motor_RM_t *motor, uint8_t *rxBuf);
void RM_Set_Torque(Motor_RM_Set_t *set);

#endif
