#include "motor.h"


/**
 * @brief 此文件都是赋值初始化
 * 
 */
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
Motor_DM_Rx_Data_t Yaw_Rx_Data;
Motor_DM_Tx_Data_t Yaw_Tx_Data;
Motor_DM_State_Data_t Yaw_State_Data;
Motor_DM_Rx_Data_t Pitch_Rx_Data;
Motor_DM_Tx_Data_t Pitch_Tx_Data;
Motor_DM_State_Data_t Pitch_State_Data;
Motor_RM_Rx_Data_t R_Fric_Rx;
Motor_RM_Tx_Data_t R_Fric_Tx;
Motor_RM_Rx_Data_t L_Fric_Rx;
Motor_RM_Tx_Data_t L_Fric_Tx;
Motor_RM_Rx_Data_t Lift_Rx;
Motor_RM_Tx_Data_t Lift_Tx;
LK_Motor_Rx_Data_t Dial_Rx_Data;
LK_Motor_Tx_Data_t Dial_Tx_Data;

pid_ctrl_t yaw_gyro_outer = { .kp = 10.0f, .ki = 0.05f, .kd = 0.0f, .integral_max = 200.0f, .out_max = 500.0f };
pid_ctrl_t yaw_gyro_inner = { .kp = 0.04f, .ki = 0.0f, .kd = 0.0f, .integral_max = 0.0f, .out_max = 5.0f };
pid_ctrl_t yaw_mec_outer = { .kp = 0.15f, .ki = 0.0f, .kd = 0.0f, .integral_max = 500.0f, .out_max = 7.0f };
pid_ctrl_t pitch_mec_outer = { .kp = 1.0f, .ki = 0.0f, .kd = 0.0f, .integral_max = 500.0f, .out_max = 10.0f };
pid_ctrl_t L_Fric_Speed_Ctrl = { .kp = 6.0f, .ki = 0.f, .kd = 0.f, .integral_max =7000.f, .out_max = 10000.f };
pid_ctrl_t R_Fric_Speed_Ctrl = { .kp = 6.0f, .ki = 0.f, .kd = 0.f, .integral_max =7000.f, .out_max = 10000.f };
pid_ctrl_t lift_speed_pid = { .kp = 20.0f, .ki = 0.0f, .kd = 0.0f, .integral_max = 0.0f, .out_max = 700.0f };
pid_ctrl_t lift_angle_outer = { .kp = 8.0f, .ki = 0.0f, .kd = 0.0f, .integral_max = 0.0f, .out_max = 300.0f };
pid_ctrl_t lift_angle_inner = { .kp = 50.0f, .ki = 0.0f, .kd = 0.0f, .integral_max = 0.0f, .out_max = 700.0f };


Motor_DM_Born_Data_t Yaw_Born_Data = { .tx_id = 0x002, .hcan = &hcan2 };
Motor_DM_Born_Data_t Pitch_Born_Data = { .tx_id = 0x001, .hcan = &hcan1 };
Motor_RM_Born_Data_t R_Fric_Born = { .rxId = 1, .hcan = &hcan1, .stdId = 0x200, .type = RM_TYPE_3508 };
Motor_RM_Born_Data_t L_Fric_Born = { .rxId = 0, .hcan = &hcan1, .stdId = 0x200, .type = RM_TYPE_3508 };
Motor_RM_Born_Data_t Lift_Born = { .rxId = 3, .hcan = &hcan1, .stdId = 0x200, .type = RM_TYPE_2006 };
LK_Motor_Born_Data_t Dial_Born_Data = { .hcan = &hcan1, .id = 1 };

Motor_RM_Ctrl_Data_t R_Fric_Ctrl = { .speed_ctrl = &R_Fric_Speed_Ctrl };
Motor_RM_Ctrl_Data_t L_Fric_Ctrl = { .speed_ctrl = &L_Fric_Speed_Ctrl };
Motor_RM_Ctrl_Data_t Lift_Ctrl = { .speed_ctrl = &lift_speed_pid };

Motor_DM_t Yaw_Motor =
{
	.born_data  = &Yaw_Born_Data,
	.rx_data    = &Yaw_Rx_Data,
	.tx_data    = &Yaw_Tx_Data,
	.state_data = &Yaw_State_Data,
};

Motor_DM_t Pitch_Motor =
{
	.born_data  = &Pitch_Born_Data,
	.rx_data    = &Pitch_Rx_Data,
	.tx_data    = &Pitch_Tx_Data,
	.state_data = &Pitch_State_Data,
};

Motor_RM_t R_Fric =
{
	.born_data = &R_Fric_Born,
	.rx_data = &R_Fric_Rx,
	.tx_data = &R_Fric_Tx,
	.ctrl_data = &R_Fric_Ctrl,
};

Motor_RM_t L_Fric =
{
	.born_data = &L_Fric_Born,
	.rx_data = &L_Fric_Rx,
	.tx_data = &L_Fric_Tx,
	.ctrl_data = &L_Fric_Ctrl,
};
Motor_RM_t Lift_Motor =
{
	.born_data = &Lift_Born,
	.rx_data = &Lift_Rx,
	.tx_data = &Lift_Tx,
	.ctrl_data = &Lift_Ctrl,
};
LK_Motor_t Dial_Motor =
{
	.born_data = &Dial_Born_Data,
	.rx_data = &Dial_Rx_Data,
	.tx_data = &Dial_Tx_Data,
};
Motor_RM_Set_t RM_Group =
{
    .motor[0] = &L_Fric,
	.motor[1] = &R_Fric,
	.motor[3] = &Lift_Motor,
    .stdId = 0x200,
	.hcan = &hcan1,
};
