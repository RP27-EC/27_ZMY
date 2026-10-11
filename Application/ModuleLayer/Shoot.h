#ifndef __SHOOT_H
#define __SHOOT_H

#include "stdint.h"


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
