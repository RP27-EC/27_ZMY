#include "Lift.h"
#include "motor.h"
#include "board.h"
#include "PID.h"
#include "math.h"

Lift_t lift =
{
	.state = LIFT_HOLDING,
	.mode  = LIFT_STOP,
};

static void Lift_Target_Update(Lift_t* lift);
static void Lift_State_Machine(Lift_t* lift);
static void Lift_Pid_Calculate(Lift_t* lift);
static void Lift_Jam_Check(Lift_t* lift);
static void Lift_Tx_Update(Lift_t* lift);
static uint8_t Lift_Get_State(void);

static uint8_t Lift_Get_State(void)
{
	if(HAL_GetTick() - Lift_Motor.rx_data->last_rx_tick > LIFT_LOST_TICK)
	{
		return LIFT_STATE_UNKNOWN;
	}

	return LIFT_STATE_ONLINE;
}

static void Lift_Target_Update(Lift_t* lift)
{
	Lift_State_e state;

	if(Board_Is_Online(&board) == 0)
	{
		return;
	}

	if(board.rx_data->shoot_target.lift == LIFT_EXTEND)
	{
		state = LIFT_TO_UP;
	}
	else if(board.rx_data->shoot_target.lift == LIFT_RETRACT)
	{
		state = LIFT_TO_DOWN;
	}
	//升降机构保持，清零堵转
	else
	{
		if(lift->state == LIFT_HOLDING && lift->mode == LIFT_ANGLE)
		{
			return;
		}

		lift->state = LIFT_HOLDING;
		lift->target_angle = Lift_Motor.rx_data->motor_angle_sum;
		lift->jam = 0;

		return;
	}
	//状态变换堵转的记录清零
	if(lift->state != state)
	{
		lift->jam = 0;
		lift->jam_tick = 0;
	}

	lift->state = state;
}
//升降状态机
static void Lift_State_Machine(Lift_t* lift)
{
	//堵转
	if(lift->jam != 0)
	{
		lift->mode = LIFT_STOP;
		return;
	}
	//上下都是速度环控制，在保持中使用角度环
	switch(lift->state)
	{
	case LIFT_TO_UP:
		lift->mode = LIFT_SPEED;
		lift->target_speed = LIFT_JOG_SPEED;
		break;

	case LIFT_TO_DOWN:
		lift->mode = LIFT_SPEED;
		lift->target_speed = -LIFT_JOG_SPEED;
		break;

	case LIFT_HOLDING:
		lift->mode = LIFT_ANGLE;
		break;

	default:
		lift->mode = LIFT_STOP;
		break;
	}
}

static void Lift_Pid_Calculate(Lift_t* lift)
{
	switch(lift->mode)
	{
	case LIFT_SPEED:
		lift->out = all_pid_calc(NULL, &lift_speed_pid, lift->target_speed, 0.f, Lift_Motor.rx_data->speed, 1.0f, 2);
		break;

	case LIFT_ANGLE:
		lift->out = all_pid_calc(&lift_angle_outer, &lift_angle_inner, lift->target_angle, Lift_Motor.rx_data->motor_angle_sum, Lift_Motor.rx_data->speed, -1.0f, 0);
		break;

	default:
		lift->out = 0.f;
		break;
	}
}
//堵转判断
static void Lift_Jam_Check(Lift_t* lift)
{
	if(lift->state != LIFT_TO_DOWN || lift->mode != LIFT_SPEED)
	{
		lift->jam_tick = 0;
		return;
	}

	if(fabsf(Lift_Motor.rx_data->speed) >= LIFT_JAM_SPEED) //速度小于堵转判断的最小速度才判断堵转
	{
		lift->jam_tick = 0;
		return;
	}

	lift->jam_tick ++;

	if(lift->jam_tick > LIFT_JAM_TICK)
	{
		lift->jam_tick = 0;
		lift->jam = 1;
		lift->mode = LIFT_STOP;
		lift->out = 0.0f;
	}
}

static void Lift_Tx_Update(Lift_t* lift)
{
	board.tx_data->lift_state = Lift_Get_State();
}

void Lift_Work(Lift_t* lift)
{
	if(Board_Is_Online(&board) == 0 || board.rx_data->car_data.car_state == CAR_SLEEP || HAL_GetTick() - Lift_Motor.rx_data->last_rx_tick > LIFT_LOST_TICK)
	{
		lift->mode = LIFT_STOP;
		lift->out = 0.0f;
		Lift_Motor.tx_data->torque = 0.0f;
		return;
	}

	Lift_Target_Update(lift);
	Lift_State_Machine(lift);
	Lift_Pid_Calculate(lift);
	Lift_Jam_Check(lift);
	Lift_Tx_Update(lift);

	Lift_Motor.tx_data->torque = lift->out;
}
