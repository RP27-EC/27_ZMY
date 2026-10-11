#ifndef __LIFT_H__
#define __LIFT_H__

#include "stdint.h"

#define LIFT_EXTEND          0
#define LIFT_RETRACT         1
#define LIFT_HOLD            2

#define LIFT_STATE_UNKNOWN   0
#define LIFT_STATE_ONLINE    1

#define LIFT_PITCH_TOL       0.15f
#define LIFT_YAW_TOL         0.15f
#define LIFT_RC_LOST_TICK    60

#define LIFT_TW_UP           (-500)
#define LIFT_TW_DOWN         500

typedef struct Lift_Struct_t
{
	uint8_t request;
	uint8_t pitch_ready;
	uint8_t yaw_ready;
	uint8_t pitch_lock;
	uint8_t yaw_lock;
}Lift_t;

extern Lift_t lift;
extern uint8_t lift_key_active;
extern uint8_t lift_lower_lock;

void Lift_Work(Lift_t* lift);

#endif
