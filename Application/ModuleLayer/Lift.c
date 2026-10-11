#include "Lift.h"
#include "board.h"
#include "gimbal.h"
#include "rc_sensor.h"
#include "rp_math.h"
#include "math.h"

Lift_t lift =
{
	.request = LIFT_EXTEND,
};

uint8_t lift_key_active = 0;
uint8_t lift_lower_lock = 0;

static void Lift_Request_Update(Lift_t* lift);
static void Lift_Lock_Update(Lift_t* lift);
static void Lift_Lower_Lock_Update(Lift_t* lift);
static void Lift_Tx_Update(Lift_t* lift);

static void Lift_Request_Update(Lift_t* lift)
{
	int16_t tw;

	lift_key_active = 0;

	if(HAL_GetTick() - rc_sensor_info.last_rx_tick > LIFT_RC_LOST_TICK)
	{
		lift->pitch_ready = 0;
		lift->yaw_ready = 0;
		lift->request = LIFT_HOLD;//保持角度
		return;
	}

	if(rc_sensor_info.s1.value == RC_SW_DOWN && (rc_sensor_info.Q.value != 0 || rc_sensor_info.E.value != 0))
	{
		lift_key_active = 1;
	}

	if(rc_sensor_info.s1.value != RC_SW_UP && lift_key_active == 0)
	{
		lift->pitch_ready = 0;
		lift->yaw_ready = 0;
		lift->request = LIFT_HOLD;
		return;
	}

	tw = rc_sensor_info.thumbwheel.value;
	//云台两个东西都回到中位了
	if(lift->pitch_ready == 0 && fabsf(board.rx_data->gimbal_data.pitch_mec - GIMBAL_PITCH_MIDDLE) <= LIFT_PITCH_TOL)
	{
		lift->pitch_ready = 1;
	}

	if(lift->yaw_ready == 0 && fabsf(motor_half_cycle(GIMBAL_YAW_FRONT - board.rx_data->gimbal_data.yaw_mec, 2.f * PI)) <= LIFT_YAW_TOL)
	{
		lift->yaw_ready = 1;
	}

	if(lift_key_active != 0)
	{
		if(rc_sensor_info.Q.value != 0)
		{
			lift->request = LIFT_EXTEND;//上升
		}
		else
		{
			lift->request = (lift->pitch_ready != 0 && lift->yaw_ready != 0) ? LIFT_RETRACT : LIFT_HOLD;
		}

		return;
	}

	if(tw >= LIFT_TW_DOWN)
	{
		lift->request = (lift->pitch_ready != 0 && lift->yaw_ready != 0) ? LIFT_RETRACT : LIFT_HOLD;
		return;
	}

	lift->pitch_ready = 0;
	lift->yaw_ready = 0;

	lift->request = (tw <= LIFT_TW_UP) ? LIFT_EXTEND : LIFT_HOLD;
}
//S1为UP时底盘云台关闭控制
static void Lift_Lock_Update(Lift_t* lift)
{
	uint8_t lock = (rc_sensor_info.s1.value == RC_SW_UP || lift_key_active != 0) ? 1 : 0;

	lift->pitch_lock = lock;
	lift->yaw_lock = lock;
}

static void Lift_Lower_Lock_Update(Lift_t* lift)
{
	if(lift->request == LIFT_RETRACT)
	{
		lift_lower_lock = 1;
	}
	else if(lift->request == LIFT_EXTEND)
	{
		lift_lower_lock = 0;
	}
}

static void Lift_Tx_Update(Lift_t* lift)
{
	board_tx_data.shoot_data.lift = lift->request;
}

void Lift_Work(Lift_t* lift)
{
	Lift_Request_Update(lift);
	Lift_Lock_Update(lift);
	Lift_Lower_Lock_Update(lift);
	Lift_Tx_Update(lift);
}
