#ifndef __CHASSIS_H
#define __CHASSIS_H

#include  "motor.h"

#define   WHEEL_MAX_RPM         4000
#define   WHEEL_MAX_SPEED       (WHEEL_MAX_RPM / _3508_REDUCT_RATIO / 60.f * 2.f * 3.14159265f)

#define   CHASSIS_MAX_SPEED     WHEEL_MAX_SPEED
#define   FRONT_MAX_SPEED       50
#define   LEFT_MAX_SPEED        50
#define   CYCLE_MAX_SPEED       40
#define   TURN_CYCLE_SPEED      8.2f
#define   CHASSIS_FOLLOW_KP     20.f
#define   CHASSIS_FOLLOW_MIN    5.f
#define   CHASSIS_FOLLOW_MAX    16.5f
#define   CHASSIS_FOLLOW_DEAD_RAD  0.035f
#define   CHASSIS_MOUSE_FULL     10.f
#define   CHASSIS_MOUSE_DECAY    0.98f
#define   CHASSIS_MOUSE_MIN      0.05f

#define   RC_LOST_TICK          60
#define   WHEEL_LOST_TICK       20

#define  ROOM_ENOUGH_GIMBAL   0

typedef struct{
	float front_speed;
	float left_speed;
	float cycle_speed;

	float motor_speed[WHEEL_CNT];
}Chassis_Target_t;

typedef struct{
	float wheel_initial_out[WHEEL_CNT];
	float wheel_end_out[WHEEL_CNT];
}Chassis_Out_t;

typedef struct Chassis_Struct_t{
	Motor_RM_Set_t*   wheel;
	Chassis_Target_t    target;
	Chassis_Out_t       out;
}Chassis_t;

extern Chassis_t chassis;
extern uint8_t spin_key_active;

void Chassis_Work(Chassis_t* chassis);

#endif
