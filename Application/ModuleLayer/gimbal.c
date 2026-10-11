#include "gimbal.h"
#include "board.h"
#include "motor.h"
#include "imu_sensor.h"
#include "PID.h"
#include "rp_math.h"
#include "math.h"

static void Gimbal_Target_Update(Gimbal_t* gimbal);
static void Gimbal_Mec_Data_Update(Gimbal_t* gimbal);
static void Gimbal_Tx_Data_Update(Gimbal_t* gimbal);
static uint8_t Gimbal_Should_Coast(Gimbal_t* gimbal);
static void Gimbal_Torque_Zero(Gimbal_t* gimbal);
static void Gimbal_Pid_Calculate(Gimbal_t* gimbal);
static void Gimbal_Cmd_Transmit(Gimbal_t* gimbal);

Gimbal_t gimbal;

static float pitch_speed_filter = 0.f;
static float yaw_speed_filter = 0.f;

static void Gimbal_Target_Update(Gimbal_t* gimbal)
{
	if(Board_Is_Online(&board) == 0)
	{
		return;
	}

	if(board.rx_data->car_data.car_state == CAR_SLEEP)
	{
		return;
	}

	gimbal->target.yaw_target = board.rx_data->gimbal_target_data.yaw_imu_tar;

	gimbal->target.yaw_mec_target = board.rx_data->gimbal_target_data.yaw_mec_tar / PI * 180.f;
	gimbal->target.yaw_mec_target = motor_half_cycle(gimbal->target.yaw_mec_target, 360.f);

	gimbal->target.pitch_target = constrain(board.rx_data->gimbal_target_data.pitch_mec_tar / PI * 180.f, PITCH_MEC_MIN_ANGLE, PITCH_MEC_MAX_ANGLE);
	gimbal->target.pitch_target = motor_half_cycle(gimbal->target.pitch_target, 360.f);
}

static void Gimbal_Mec_Data_Update(Gimbal_t* gimbal)
{
	float pitch_speed_raw = Pitch_Motor.rx_data->speed;
	float yaw_speed_raw = Yaw_Motor.rx_data->speed;

	gimbal->pitch_mec_angle = motor_half_cycle(Pitch_Motor.rx_data->motor_angle_sum - PITCH_MOTOR_ANGLE_MIDDLE, 2.f * PI);
	//如果速度小于死区就不计算了
	if(pitch_speed_raw < PITCH_SPEED_DEADBAND && pitch_speed_raw > -PITCH_SPEED_DEADBAND)
	{
		pitch_speed_raw = 0.f;
	}

	if(yaw_speed_raw < YAW_SPEED_DEADBAND && yaw_speed_raw > -YAW_SPEED_DEADBAND)
	{
		yaw_speed_raw = 0.f;
	}

	pitch_speed_filter = PITCH_SPEED_FILTER_ALPHA * pitch_speed_raw + (1.f - PITCH_SPEED_FILTER_ALPHA) * pitch_speed_filter; //都作为阻尼项的计算项，低通滤波
	yaw_speed_filter = YAW_SPEED_FILTER_ALPHA * (yaw_speed_raw / YAW_MEC_GEAR) + (1.f - YAW_SPEED_FILTER_ALPHA) * yaw_speed_filter;
}

float Yaw_Mec_Reduce(float angle)
{
	return motor_half_cycle(angle / YAW_MEC_GEAR, 2.f * PI);
}
//发给下板的数据集
static void Gimbal_Tx_Data_Update(Gimbal_t* gimbal)
{
	board.tx_data->gimbal_data.yaw_imu   = imu_sensor.info->base_info.yaw;
	board.tx_data->gimbal_data.pitch_imu = imu_sensor.info->base_info.pitch;
	gimbal->yaw_mec_angle = Yaw_Mec_Reduce(Yaw_Motor.rx_data->motor_angle);
	board.tx_data->gimbal_data.yaw_mec   = gimbal->yaw_mec_angle;
	board.tx_data->gimbal_data.pitch_mec = gimbal->pitch_mec_angle;
}

static uint8_t Gimbal_Should_Coast(Gimbal_t* gimbal)
{
	if(Board_Is_Online(&board) == 0)
	{
		return 1;
	}

	if(board.rx_data->car_data.car_state == CAR_SLEEP)
	{
		return 1;
	}

	return 0;
}

static void Gimbal_Torque_Zero(Gimbal_t* gimbal)
{
	gimbal->out.yaw_out = 0.f;
	gimbal->out.pitch_out = 0.f;

	if(Board_Is_Online(&board) != 0 && DM_Motor_Is_Ready(&Yaw_Motor) == 0)
	{
		DM_Set_Torque(&Yaw_Motor);
		DM_Set_Torque(&Pitch_Motor);
		return;
	}

	DM_Motor_Coast(&Yaw_Motor);
	DM_Motor_Coast(&Pitch_Motor);
}

static void Gimbal_Pid_Calculate(Gimbal_t* gimbal)
{
	if(board.rx_data->gimbal_target_data.yaw_mode == 0)
	{
		yaw_mec_outer.err = motor_half_cycle(gimbal->target.yaw_mec_target - gimbal->yaw_mec_angle / PI * 180.f, 360.f);
		single_pid_ctrl(&yaw_mec_outer);

		gimbal->out.yaw_out = yaw_mec_outer.out - YAW_MEC_DAMPING * yaw_speed_filter;//在机械模式下云台速度比较快，加入一个阻尼项对角速度进行计算，防超调，与pitch类似
	}
	//陀螺仪档的双环控制
	else
	{
		float err = motor_half_cycle(gimbal->target.yaw_target - imu_sensor.info->base_info.yaw, 360.f);
		//角度环
		yaw_gyro_outer.err = err;
		single_pid_ctrl(&yaw_gyro_outer);
		
		//速度环
		yaw_gyro_inner.target = yaw_gyro_outer.out;
		yaw_gyro_inner.err = yaw_gyro_inner.target - imu_sensor.info->base_info.rate_yaw;
		single_pid_ctrl(&yaw_gyro_inner);

		gimbal->out.yaw_out = yaw_gyro_inner.out;
	}

	pitch_mec_outer.err = motor_half_cycle(gimbal->target.pitch_target - gimbal->pitch_mec_angle / PI * 180.f, 360.f);
	single_pid_ctrl(&pitch_mec_outer);

	gimbal->out.pitch_out = pitch_mec_outer.out - PITCH_MEC_DAMPING * pitch_speed_filter;//因为重力前馈的方案，写的不太好，所以采用PID加阻尼的方案来维持炮筒
}


static void Gimbal_Cmd_Transmit(Gimbal_t* gimbal)
{
	if(DM_Motor_Is_Ready(&Pitch_Motor) == 0)
	{
		gimbal->out.pitch_out = 0.f;
	}

	Yaw_Motor.tx_data->torque = gimbal->out.yaw_out;

	DM_Set_Torque(&Yaw_Motor);

	gimbal->out.pitch_out = constrain(gimbal->out.pitch_out, -GIMBAL_PITCH_TORQUE_MAX, GIMBAL_PITCH_TORQUE_MAX);

	Pitch_Motor.tx_data->torque = gimbal->out.pitch_out;
	Pitch_Motor.tx_data->target_speed = 0.f;
	Pitch_Motor.tx_data->Kd = 0.f;

	DM_Set_Torque(&Pitch_Motor);
}


void Gimbal_Work(Gimbal_t* gimbal)
{
	Gimbal_Target_Update(gimbal);
	Gimbal_Mec_Data_Update(gimbal);
	Gimbal_Tx_Data_Update(gimbal);

	if(Gimbal_Should_Coast(gimbal) != 0)
	{
		gimbal->work_flag = 0;
		Gimbal_Torque_Zero(gimbal);
		return;
	}

	if(gimbal->work_flag == 0)
	{
		gimbal->work_flag = 1;

		yaw_gyro_outer.integral = 0.f;
		yaw_gyro_inner.integral = 0.f;

		yaw_mec_outer.integral = 0.f;

		pitch_mec_outer.integral = 0.f;
		pitch_speed_filter = 0.f;
		yaw_speed_filter = 0.f;
	}

	Gimbal_Pid_Calculate(gimbal);

	Gimbal_Cmd_Transmit(gimbal);
}
