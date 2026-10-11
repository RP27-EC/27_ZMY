#include "pid.h"
#include "rp_math.h"

void single_pid_ctrl(pid_ctrl_t *pid)
{
    // 保存误差值(需要在外面自行计算误差)
	//pid->err = pid->target-pid->measure;
	pid->integral += pid->err;  
    pid->integral = constrain(pid->integral, -pid->integral_max, +pid->integral_max);
    // p i d 输出项计算
    pid->pout = pid->kp * pid->err;
    pid->iout = pid->ki * pid->integral;
	pid->last_dout=pid->dout;
    // 累加pid输出值
    pid->out = pid->pout + pid->iout + pid->dout;
    pid->out = constrain(pid->out, -pid->out_max, pid->out_max);
    // 记录上次误差值
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


	single_pid_ctrl(out);

	inn->target = out->out;
	inn->measure = mea_in * inner_kp;
	inn->err = inn->target + inn->measure;
	single_pid_ctrl(inn);

	return inn->out;
}