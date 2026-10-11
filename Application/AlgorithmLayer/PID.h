#ifndef __PID_H
#define __PID_H
#include "main.h"


#define ERR_MODE_NONE       0
#define ERR_MODE_8191       1
#define ERR_MODE_8191_4095  2
#define ERR_MODE_360        3
#define ERR_MODE_65535      4
#define ERR_MODE_191        5

typedef struct pid_ctrl {
	float	  target;
	float 	measure;
	float 	err;
	float 	last_err;
	float	  kp;
	float 	ki;
	float 	kd;
	float 	pout;
	float 	iout;
	float 	dout;
	float 	out;
	float  	last_dout;
	float	  integral;
	float 	integral_max;
	float 	out_max;

} pid_ctrl_t;

void integral_to_zero(pid_ctrl_t *pid);
void single_pid_ctrl(pid_ctrl_t *pid);
float all_pid_calc(pid_ctrl_t *out, pid_ctrl_t *inn, float target, float mea_out, float mea_in, float inner_kp, uint8_t err_cal_mode);


#endif
