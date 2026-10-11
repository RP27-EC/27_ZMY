#include "motor.h"

/**
 * @brief 此文件用于初始化赋值
 * 
 */
pid_ctrl_t wheel_speed_pid[WHEEL_CNT] = {
	[WHEEL_LF] = { .kp = 0.15f, .ki = 0, .kd = 0, .integral_max = 0, .out_max = 2.1f },
    [WHEEL_LB] = { .kp = 0.15f, .ki = 0, .kd = 0, .integral_max = 0, .out_max = 2.1f },
	[WHEEL_RF] = { .kp = 0.15f, .ki = 0, .kd = 0, .integral_max = 0, .out_max = 2.1f },
	[WHEEL_RB] = { .kp = 0.15f, .ki = 0, .kd = 0, .integral_max = 0, .out_max = 2.1f },
};

Motor_RM_Born_Data_t wheel_born_data[WHEEL_CNT] = {
	[WHEEL_LF] = { .rxId = 0, .stdId = 0x200, .hcan = &hfdcan1 },[WHEEL_LB] = { .rxId = 1, .stdId = 0x200, .hcan = &hfdcan1 },
	[WHEEL_RF] = { .rxId = 2, .stdId = 0x200, .hcan = &hfdcan1 },[WHEEL_RB] = { .rxId = 3, .stdId = 0x200, .hcan = &hfdcan1 },
};

Motor_RM_Rx_Data_t   wheel_rx_data[WHEEL_CNT];
Motor_RM_Tx_Data_t   wheel_tx_data[WHEEL_CNT];

Motor_RM_Ctrl_Data_t wheel_ctrl_data[WHEEL_CNT] = {
	[WHEEL_LF] = {
		.speed_ctrl = &wheel_speed_pid[WHEEL_LF],
	},
	[WHEEL_LB] = {
		.speed_ctrl = &wheel_speed_pid[WHEEL_LB],
	},
	[WHEEL_RF] = {
		.speed_ctrl = &wheel_speed_pid[WHEEL_RF],
	},
	[WHEEL_RB] = {
		.speed_ctrl = &wheel_speed_pid[WHEEL_RB],
	},
};

Motor_RM_t wheel_motor[WHEEL_CNT] = {
	[WHEEL_LF] = {
		.born_data = &wheel_born_data[WHEEL_LF],
		.rx_data = &wheel_rx_data[WHEEL_LF],
		.tx_data = &wheel_tx_data[WHEEL_LF],
		.ctrl_data = &wheel_ctrl_data[WHEEL_LF],
	},
	[WHEEL_LB] = {
		.born_data = &wheel_born_data[WHEEL_LB],
		.rx_data = &wheel_rx_data[WHEEL_LB],
		.tx_data = &wheel_tx_data[WHEEL_LB],
		.ctrl_data = &wheel_ctrl_data[WHEEL_LB],
	},
	[WHEEL_RF] = {
		.born_data = &wheel_born_data[WHEEL_RF],
		.rx_data = &wheel_rx_data[WHEEL_RF],
		.tx_data = &wheel_tx_data[WHEEL_RF],
		.ctrl_data = &wheel_ctrl_data[WHEEL_RF],
	},
	[WHEEL_RB] = {
		.born_data = &wheel_born_data[WHEEL_RB],
		.rx_data = &wheel_rx_data[WHEEL_RB],
		.tx_data = &wheel_tx_data[WHEEL_RB],
		.ctrl_data = &wheel_ctrl_data[WHEEL_RB],
	},
};

Motor_RM_Set_t wheel_set = {
	.motor[WHEEL_LF] = &wheel_motor[WHEEL_LF],
	.motor[WHEEL_LB] = &wheel_motor[WHEEL_LB],
	.motor[WHEEL_RF] = &wheel_motor[WHEEL_RF],
	.motor[WHEEL_RB] = &wheel_motor[WHEEL_RB],
	.stdId = 0x200,
	.hcan = &hfdcan1,
};
