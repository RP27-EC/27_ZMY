#include "chassis.h"
#include "rc_sensor.h"
#include "config_chassis.h"
#include "rp_math.h"
#include "board.h"
#include "pid.h"
#include "gimbal.h"
#include "Lift.h"

static void Chassis_Target_Clear(Chassis_t* chassis);
static uint8_t Chassis_Should_Coast(Chassis_t* chassis);
static void Chassis_Torque_Zero(Chassis_t* chassis);
static float Chassis_Follow_Calc(void);
static void Chassis_Target_Update(Chassis_t* chassis);
static void Chassis_Inverse_Calculate(Chassis_t* chassis);
static void Chassis_Pid_Calculate(Chassis_t* chassis);
static void Chassis_Cmd_Transmit(Chassis_t* chassis);

int8_t chassis_wheel_sign[WHEEL_CNT] = {1, 1, 1, 1}; //这个对功能没什么影响，是我看教学的时候说防止轮装错了或者移植时比较好写。

uint8_t spin_key_active = 0; //键鼠操作下小陀螺的标志位

static float chassis_mouse_cycle = 0.0f; //键鼠下（机械模式）底盘的Vx转向变量

Chassis_t chassis = {.wheel = &wheel_set,};  //motor.c里面集中赋值， 每个带电机的功能都是差不多架构，Hardware里面写电机驱动，在module里面写实现逻辑，motor里面进行初始化的操作（存储PID参数，还有CAN的ID等）

/**
 * @brief 目标值清零在关控中会使用
 * 
 * @param chassis 
 */
static void Chassis_Target_Clear(Chassis_t* chassis)
{
	chassis->target.front_speed = 0;
	chassis->target.left_speed = 0;
	chassis->target.cycle_speed = 0;

	for(uint8_t i = 0; i < WHEEL_CNT; i++)
	{
		chassis->target.motor_speed[i] = 0;
	}
}

/**
 * @brief 底盘电机使不使能的状态
 * 
 * @param chassis 
 * @return uint8_t 
 */
static uint8_t Chassis_Should_Coast(Chassis_t* chassis)
{
	if(rc_lost_lock != 0) //遥控器关控，然后因为设计是S1处于DOWM时全车使能，加这个是因为需要在车关控之后开控时先把S1拨到MID再拨回DOWM才能使能，防上电疯车了，S1在MID时是全车电机发0的
	{
		return 1;
	}

	if(lift_key_active != 0) //键鼠模式下升降正在执行
	{
		return 1;
	}

	if(HAL_GetTick() - rc_sensor_info.last_rx_tick <= RC_LOST_TICK && rc_sensor_info.s1.value == RC_SW_UP) //升降档时底盘不使能
	{
		return 1;
	}

	for(uint8_t i = 0; i < WHEEL_CNT; i++)
	{
		if(HAL_GetTick() - chassis->wheel->motor[i]->rx_data->last_rx_tick > WHEEL_LOST_TICK) //电机失联检测
		{
			return 1;
		}
	}

	return 0;
}

/**
 * @brief 电机发0卸力
 * 
 * @param chassis 
 */
static void Chassis_Torque_Zero(Chassis_t* chassis)
{
	for(uint8_t i = 0; i < WHEEL_CNT; i++)
	{
		chassis->wheel->motor[i]->tx_data->torque = 0;
		chassis->wheel->motor[i]->tx_data->torque_current = 0;
		chassis->wheel->motor[i]->tx_data->torque_current_raw = 0;
		chassis->out.wheel_initial_out[i] = 0;
		chassis->out.wheel_end_out[i] = 0;
	}

	RM_Set_Torque(chassis->wheel);
}

/**
 * @brief 陀螺仪模式下的跟随环
 * 
 * @return float 
 */
static float Chassis_Follow_Calc(void)
{
	float yaw_err = motor_half_cycle(board.rx_data->gimbal_data.yaw_mec - GIMBAL_YAW_FRONT, 2.f * PI); //云台转过的角度
	float cycle = 0.0f;

	if(Board_Is_Online(&board) != 0  && fabsf(yaw_err) > CHASSIS_FOLLOW_DEAD_RAD)
	{
		cycle = constrain(CHASSIS_FOLLOW_KP * fabsf(yaw_err) + CHASSIS_FOLLOW_MIN, 0.f, CHASSIS_FOLLOW_MAX);
		cycle = (yaw_err > 0.0f) ? -cycle : cycle;
	}

	return cycle;
}

static void Chassis_Target_Update(Chassis_t* chassis)
{
	spin_key_active = 0;

	if(HAL_GetTick() - rc_sensor_info.last_rx_tick > RC_LOST_TICK)
	{
		Chassis_Target_Clear(chassis);
		return;
	}

	switch(rc_sensor_info.s1.value)
	{
		case RC_SW_DOWN:
			break;

		case RC_SW_MID:
		case RC_SW_UP:
		default:
			Chassis_Target_Clear(chassis);
			return;
	}

	//在键鼠下只有在升降上一状态不是下降并且不在机械模式，并且按下了SHIFT才能进入小陀螺
	spin_key_active = (rc_sensor_info.Shift.value != 0 && gimbal.target.yaw_mode != 0 && lift_lower_lock == 0) ? 1 : 0;

	if(rc_sensor_info.s2.value == RC_SW_MID && gimbal.target.yaw_mode == 0 && spin_key_active == 0) //机械模式
	{
		if(rc_sensor_info.mouse_vx != 0)
		{
			chassis_mouse_cycle = (float)rc_sensor_info.mouse_vx / CHASSIS_MOUSE_FULL * TURN_CYCLE_SPEED;
			rc_sensor_info.mouse_vx = 0;
		}
		else
		{
			chassis_mouse_cycle *= CHASSIS_MOUSE_DECAY;//上一值*0.98作为鼠标停下后的值，使得控制比较平滑，不会出现鼠标停下后车突然顿一下的状况

			if(fabsf(chassis_mouse_cycle) < CHASSIS_MOUSE_MIN) //小于最小判断为停下了
			{
				chassis_mouse_cycle = 0.0f;
			}
		}
	}
	else
	{
		chassis_mouse_cycle = 0.f;
	}

	float vx = -(RC_LEFT_CH_UD_VALUE / 660.f + (float)(rc_sensor_info.W.value - rc_sensor_info.S.value)) * FRONT_MAX_SPEED;
	float vy = (RC_LEFT_CH_LR_VALUE / 660.f - (float)(rc_sensor_info.A.value - rc_sensor_info.D.value)) * LEFT_MAX_SPEED;
	float yaw = 0.f;

	//在小陀螺模式下，不能转头在车尾，不能上一状态是云台下降
  	if ((rc_sensor_info.s2.value == RC_SW_UP || gimbal.turn_back != 0 || spin_key_active != 0) && lift_lower_lock == 0)
  	{
		yaw = board.rx_data->gimbal_data.yaw_mec - GIMBAL_YAW_FRONT; //云台和车头正前方夹角，通过速度分解算出在云台坐标系下的速度
        float cos = cosf(yaw);
        float sin = sinf(yaw);

        chassis->target.front_speed = vx * cos - vy * sin;
        chassis->target.left_speed  = vx * sin + vy * cos;
  	}
	else
    {
        chassis->target.front_speed = vx;
        chassis->target.left_speed  = vy;
    }
	//小陀螺的速度
    if ((rc_sensor_info.s2.value == RC_SW_UP || spin_key_active != 0) && gimbal.target.yaw_mode != 0 && lift_lower_lock == 0)
  	{
      chassis->target.cycle_speed = TURN_CYCLE_SPEED;
  	}
	//机械模式
  	else if (rc_sensor_info.s2.value == RC_SW_MID)
 	{
		if(gimbal.target.yaw_mode != 0)  //这是为键鼠准备的，键鼠不好改变键值，键鼠里面是G键改变机械和陀螺仪模式，但是单摇杆MID是机械模式，所以只能在机械模式里面套一层模式判断做到键鼠优先
		{
			chassis->target.cycle_speed = Chassis_Follow_Calc();
		}
		else if(chassis_mouse_cycle != 0.0f) //键鼠优先屏蔽摇杆了
		{
			chassis->target.cycle_speed = chassis_mouse_cycle; 
		}
		else
		{
			chassis->target.cycle_speed = RC_RIGH_CH_LR_VALUE / 660.f * TURN_CYCLE_SPEED;
		}
  	}
	//陀螺仪模式
  	else if (rc_sensor_info.s2.value == RC_SW_DOWN && lift_lower_lock == 0)
  	{
		chassis->target.cycle_speed = Chassis_Follow_Calc();
  	}
  	else
  	{
    	chassis->target.cycle_speed = 0;
  	}
}

static void Chassis_Inverse_Calculate(Chassis_t* chassis)
{
	float front = chassis->target.front_speed;
	float left = chassis->target.left_speed;
	float cycle = chassis->target.cycle_speed;

	float trans = abs(front) + abs(left);
	float rotate = abs(cycle);

	if(trans + rotate > CHASSIS_MAX_SPEED)//限幅，在超过最大速度之后，平移只能分到电机资源的40%，其余用于自转或者转向
	{
		float rotate_limit = CHASSIS_MAX_SPEED * 0.60f;

		if(rotate > rotate_limit)
		{
			cycle *= rotate_limit / rotate;
		}

		if(trans > 0.f)
		{
			float k = (CHASSIS_MAX_SPEED - abs(cycle)) / trans;

			if(k < 1.f)
			{
				front *= k;
				left *= k;
			}
		}
	}

	chassis->target.front_speed = front;
	chassis->target.left_speed = left;
	chassis->target.cycle_speed = cycle;

	chassis->target.motor_speed[WHEEL_LF] = (- front + left + cycle) * chassis_wheel_sign[WHEEL_LF];
	chassis->target.motor_speed[WHEEL_LB] = (- front - left + cycle) * chassis_wheel_sign[WHEEL_LB];
	chassis->target.motor_speed[WHEEL_RF] = (  front + left + cycle) * chassis_wheel_sign[WHEEL_RF];
	chassis->target.motor_speed[WHEEL_RB] = (  front - left + cycle) * chassis_wheel_sign[WHEEL_RB];

	for(uint8_t i = 0; i < WHEEL_CNT; i++)
	{
		chassis->target.motor_speed[i] = constrain(chassis->target.motor_speed[i], -WHEEL_MAX_SPEED, WHEEL_MAX_SPEED);
	}
}

/**
 * @brief PID计算
 * 
 * @param chassis 
 */
static void Chassis_Pid_Calculate(Chassis_t* chassis)
{
	for(uint8_t i = 0; i < WHEEL_CNT; i++)
	{
		chassis->out.wheel_initial_out[i] = all_pid_calc(NULL,
			chassis->wheel->motor[i]->ctrl_data->speed_ctrl,
			chassis->target.motor_speed[i],
			0,
			chassis->wheel->motor[i]->rx_data->speed,
			0,
			ERR_MODE_NONE);
	}
}

static void Chassis_Cmd_Transmit(Chassis_t* chassis)
{
	for(uint8_t i = 0; i < WHEEL_CNT; i++)
	{
		chassis->out.wheel_end_out[i] = chassis->out.wheel_initial_out[i];
		chassis->wheel->motor[i]->tx_data->torque = chassis->out.wheel_end_out[i];
	}

	RM_Set_Torque(chassis->wheel);
}

void Chassis_Work(Chassis_t* chassis)
{
	Chassis_Target_Update(chassis);

	if(Chassis_Should_Coast(chassis) != 0)
	{
		Chassis_Torque_Zero(chassis);
		return;
	}

	Chassis_Inverse_Calculate(chassis);
	Chassis_Pid_Calculate(chassis);
	Chassis_Cmd_Transmit(chassis);
}
