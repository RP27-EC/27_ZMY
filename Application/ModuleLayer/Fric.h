#ifndef __FRIC_H__
#define __FRIC_H__

#include "PID.h"
#include "rp_math.h"
#include "RM_motor.h"
#include "Motor.h"

#define FRIC_CNT 2
#define FRIC_MOTOR_L 0
#define FRIC_MOTOR_R 1

#define FRIC_TARGET_SPEED 1000.f
#define FRIC_READY_SPEED  (FRIC_TARGET_SPEED * 0.4f)
#define FRIC_SPEED_RAMP 15.f
#define FRIC_SPEED_FILTER_K 0.03f
#define FRIC_LOST_TICK 120
#define FRIC_RAW_TO_TORQUE  (MAX_CURRENT_3508 * TORQUE_CONSTANT_3508 / 16384.f)
typedef struct Fric_Target_Struct_t
{
    float speed;

    uint8_t enable;
} Fric_Target_t;

typedef struct Fric_Out_Struct_t
{
    float Out[FRIC_CNT];   
} Fric_Out_t;

typedef struct Fric_Struct_t
{
    Fric_Target_t target;

    Fric_Out_t out;

} Fric_t;

extern Fric_t Fric;
extern float  fric_Motor_Sign[FRIC_CNT];

void Fric_Work(Fric_t *fric);
uint8_t Fric_Is_Online(void);
uint8_t Fric_Is_Ready(void);

#endif


