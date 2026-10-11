#ifndef __MOTOR_H
#define __MOTOR_H

#include "rp_config.h"
#include "can_protocol.h"
#include "rm_motor.h"
#include "DM_motor.h"
#include "drv_can.h"
#include "LK_Motor.h"


#define ID_GIMB_Y 		0x012
#define ID_GIMB_P 		0x011
#define ID_R_FRIC 		0x202
#define ID_L_FRIC 		0x201
#define ID_DIAL 		0x141
#define ID_LIFT 		0x204

extern  Motor_DM_t Yaw_Motor;
extern  Motor_DM_t Pitch_Motor;
extern  Motor_RM_t R_Fric;
extern  Motor_RM_t L_Fric;
extern  Motor_RM_t Lift_Motor;
extern  Motor_RM_Set_t RM_Group;
extern  LK_Motor_t Dial_Motor;

extern  pid_ctrl_t yaw_gyro_outer;
extern  pid_ctrl_t yaw_gyro_inner;
extern  pid_ctrl_t yaw_mec_outer;
extern  pid_ctrl_t pitch_mec_outer;
extern  pid_ctrl_t lift_speed_pid;
extern  pid_ctrl_t lift_angle_outer;
extern  pid_ctrl_t lift_angle_inner;


#endif

