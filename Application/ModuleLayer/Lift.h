#ifndef __LIFT_H__
#define __LIFT_H__

#include "stdint.h"

#define LIFT_EXTEND          0
#define LIFT_RETRACT         1
#define LIFT_HOLD            2

#define LIFT_JOG_SPEED       250.f

#define LIFT_JAM_SPEED       10.f
#define LIFT_JAM_TICK        600

#define LIFT_LOST_TICK       120

#define LIFT_STATE_UNKNOWN   0
#define LIFT_STATE_ONLINE    1

typedef enum
{
	LIFT_TO_DOWN = 0,
	LIFT_TO_UP,
	LIFT_HOLDING,
}Lift_State_e;

typedef enum
{
	LIFT_STOP = 0,
	LIFT_SPEED,
	LIFT_ANGLE,
}Lift_Mode_e;

typedef struct Lift_Struct_t
{
	Lift_State_e state;
	Lift_Mode_e  mode;

	float target_angle;
	float target_speed;
	float out;

	uint16_t jam_tick;
	uint8_t  jam;
}Lift_t;

extern Lift_t lift;

void Lift_Work(Lift_t* lift);

#endif
