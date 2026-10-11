#include "pid.h"
#include "rp_math.h"

void single_pid_ctrl(pid_ctrl_t *pid)
{
	pid->integral += pid->err;
	pid->integral = constrain(pid->integral, -pid->integral_max, pid->integral_max);

	pid->pout = pid->kp * pid->err;
	pid->iout = pid->ki * pid->integral;
	pid->dout = pid->kd * (pid->err - pid->last_err);
	pid->last_dout = pid->dout;

	pid->out = pid->pout + pid->iout + pid->dout;
	pid->out = constrain(pid->out, -pid->out_max, pid->out_max);
	pid->last_err = pid->err;
}

float all_pid_calc(pid_ctrl_t *out, pid_ctrl_t *inn, float target, float mea_out, float mea_in, float inner_kp, uint8_t err_cal_mode)
{
	if(inn == NULL)
	{
		return 0;
	}

	if(out == NULL)
	{
		inn->target = target;
		inn->measure = mea_in;
		inn->err = inn->target - inn->measure;
		single_pid_ctrl(inn);
		return inn->out;
	}

	out->target = target;
	out->measure = mea_out;
	out->err = out->target - out->measure;

	switch(err_cal_mode)
	{
		case ERR_MODE_8191:
			out->err = motor_half_cycle(out->err, 8191);
			break;

		case ERR_MODE_65535:
			out->err = motor_half_cycle(out->err, 65535);
			break;


		default:
			break;
	}

	single_pid_ctrl(out);

	inn->target = out->out;
	inn->measure = mea_in * inner_kp;
	inn->err = inn->target + inn->measure;
	single_pid_ctrl(inn);

	return inn->out;
}
