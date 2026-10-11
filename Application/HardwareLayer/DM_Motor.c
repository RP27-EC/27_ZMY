/* Includes ------------------------------------------------------------------*/
#include "DM_Motor.h"
//达妙电机电机指令
static uint8_t Motor_Command[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};

static void Motor_Send_Data(Motor_DM_t *motor, uint8_t* buf);
static void Motor_SetControlPara(Motor_DM_t *motor);
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits);
static float uint_to_float(uint16_t x_int, float x_min, float x_max, uint8_t bits);
static void Motor_Send_Command(Motor_DM_t *motor, Motor_MIT_Command_e Command);
static void Motor_Angle_Sum_Cal(Motor_DM_t *motor);

void DM_Motor_Sleep(Motor_DM_t *motor)
{
	if(motor != NULL)
	{
		motor->tx_data->torque = 0;
		motor->tx_data->Kd = 0;
		motor->tx_data->Kp = 0;
	}
}

uint8_t DM_Motor_Is_Ready(Motor_DM_t *motor)
{
	if(motor->rx_data->last_rx_tick == 0)
	{
		return 0;
	}

	if(HAL_GetTick() - motor->rx_data->last_rx_tick > DM_MOTOR_LOST_TICK)
	{
		return 0;
	}

	if(motor->state_data->err_code != DM_Enable)
	{
		return 0;
	}

	return 1;
}
//DM电机是否能正常进入使能
void DM_Motor_Coast(Motor_DM_t *motor)
{
	motor->tx_data->torque = 0.f;

	if(motor->state_data->err_code == DM_Enable)
	{
		DM_Set_Torque(motor);
	}
	else
	{
		DM_Motor_Sleep(motor);
	}
}

void DM_Set_Torque(Motor_DM_t *motor)
{
	if(motor != NULL)
	{
		if(motor->state_data->err_code == DM_Disable)//DM电机使能
		{
			Motor_Send_Command(motor, Enter_Motor_Mode);
		}
		else if(motor->state_data->err_code == DM_Enable)
		{
			Motor_DM_Tx_Data_t* motor_tx_data = motor->tx_data;
			motor_tx_data->Kp = 0;
			motor_tx_data->Kd = 0;
			Motor_SetControlPara(motor);
			motor->tx_data->torque = 0;
		}
		else //发生错误
		{
			Motor_Send_Command(motor, Clear_Err_Code);
		}
	}
}
//接收反馈
void DM_Motor_Update(Motor_DM_t *motor, uint8_t *rxBuf)
{
	Motor_DM_Rx_Data_t* motor_rx_data = motor->rx_data;

	motor->state_data->err_code = (Motor_DM_State_e)(rxBuf[0] >> 4);
	motor_rx_data->motor_angle = uint_to_float((uint16_t)((rxBuf[1] << 8) | rxBuf[2]), P_MIN, P_MAX, 16);
	motor_rx_data->speed = uint_to_float((uint16_t)((rxBuf[3] << 4) | (rxBuf[4] >> 4)), V_MIN, V_MAX, 12);

	motor_rx_data->torque = uint_to_float((uint16_t)(((rxBuf[4]&0x0F) << 8) | rxBuf[5]), T_MIN, T_MAX, 12);

	Motor_Angle_Sum_Cal(motor);

	motor_rx_data->last_rx_tick = HAL_GetTick();
}
//跨圈计算
static void Motor_Angle_Sum_Cal(Motor_DM_t *motor)
{
	Motor_DM_Rx_Data_t* motor_rx_data = motor->rx_data;
	float angle;

	if(motor_rx_data->angle_sum_flag == 0)
	{
		motor_rx_data->angle_sum_flag = 1;
		motor_rx_data->motor_angle_last = motor_rx_data->motor_angle;
		motor_rx_data->motor_angle_sum = motor_rx_data->motor_angle;
		return;
	}

	if(HAL_GetTick() - motor_rx_data->last_rx_tick > DM_MOTOR_LOST_TICK)
	{
		motor_rx_data->motor_angle_sum = motor_rx_data->motor_angle + roundf((motor_rx_data->motor_angle_sum - motor_rx_data->motor_angle) / P_CYCLE) * P_CYCLE;
		motor_rx_data->motor_angle_last = motor_rx_data->motor_angle;
		return;
	}

	angle = motor_rx_data->motor_angle - motor_rx_data->motor_angle_last;

	if(angle > P_HALF)
	{
		angle -= P_CYCLE;
	}
	else if(angle < -P_HALF)
	{
		angle += P_CYCLE;
	}

	motor_rx_data->motor_angle_last = motor_rx_data->motor_angle;
	motor_rx_data->motor_angle_sum += angle;
}
//发指令
static void Motor_Send_Command(Motor_DM_t *motor, Motor_MIT_Command_e Command)
{
	switch(Command)
	{
		case Enter_Motor_Mode:
		Motor_Command[7] = 0xFC;
		break;
		case Clear_Err_Code:
		Motor_Command[7] = 0xFB;
		break;
		default:
		break;
	}

	Motor_Send_Data(motor, Motor_Command);
}

static void Motor_Send_Data(Motor_DM_t *motor, uint8_t* buf)
{
	Motor_DM_Born_Data_t* motor_born_data = motor->born_data;

	CAN_SendData(motor_born_data->hcan, motor_born_data->tx_id, buf);
}

static void Motor_SetControlPara(Motor_DM_t *motor)
{
	Motor_DM_Tx_Data_t* motor_tx_data = motor->tx_data;
	uint16_t p, v, kp, kd, t;
	uint8_t* buf = motor_tx_data->tx_buff;

	motor_tx_data->target_angle = constrain(motor_tx_data->target_angle, P_MIN, P_MAX);
	motor_tx_data->target_speed = constrain(motor_tx_data->target_speed, V_MIN, V_MAX); //实际上只发力矩，真正发送时这些都为0
	motor_tx_data->Kp = constrain(motor_tx_data->Kp, KP_MIN, KP_MAX);
	motor_tx_data->Kd = constrain(motor_tx_data->Kd, KD_MIN, KD_MAX);
	motor_tx_data->torque = constrain(motor_tx_data->torque, T_MIN, T_MAX);

	p = float_to_uint(motor_tx_data->target_angle,P_MIN,P_MAX, 16);
	v = float_to_uint(motor_tx_data->target_speed, V_MIN, V_MAX, 12);
	kp = float_to_uint(motor_tx_data->Kp, KP_MIN, KP_MAX, 12);
	kd = float_to_uint(motor_tx_data->Kd, KD_MIN, KD_MAX, 12);
	t = float_to_uint(motor_tx_data->torque, T_MIN,  T_MAX,  12);

	buf[0] = p>>8;
	buf[1] = p&0xFF;
	buf[2] = v>>4;
	buf[3] = ((v&0xF)<<4)|(kp>>8);
	buf[4] = kp&0xFF;
	buf[5] = kd>>4;
	buf[6] = ((kd&0xF)<<4)|(t>>8);
	buf[7] = t&0xff;

	Motor_Send_Data(motor, buf);
}

static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
	float span = x_max - x_min;
	float offset = x_min;

	return (uint16_t) ((x-offset)*((float)((1<<bits)-1))/span);
}

static float uint_to_float(uint16_t x_int, float x_min, float x_max, uint8_t bits)
{
	float span = x_max - x_min;
	float offset = x_min;

	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}
