#ifndef __SHOOT_H
#define __SHOOT_H


#include "stdint.h"

#define SHOOT_RC_TRIGGER_SIGNAL 100
#define SHOOT_RC_TRIGGER_RELEASE 70
#define SHOOT_RC_TRIGGER_CONTINUE 550
#define SHOOT_RC_LOST_TICK 60

typedef struct Shoot_Target_Struct_t
{
    uint8_t enable;
    uint8_t mode;
    uint8_t trigger;
}Shoot_Target_t;

typedef struct Shoot_Struct_t
{
    Shoot_Target_t target;
}Shoot_t;


extern Shoot_t shoot;
void Shoot_Work(Shoot_t* shoot);

#endif
