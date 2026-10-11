#ifndef __LK_MOTOR_H__
#define __LK_MOTOR_H__

#include "drv_can.h"
#include "rp_math.h"

#define LK_MOTOR_STDID 0x140
#define LK_MOTOR_CMD_CLOSE 0x80
#define LK_MOTOR_CMD_STOP 0x81

#define LK_MOTOR_CMD_CLOSE      0x80
#define LK_MOTOR_CMD_STOP       0x81
#define LK_MOTOR_CMD_ENABLE     0x88
#define LK_MOTOR_CMD_READ_ENC   0x90
#define LK_MOTOR_CMD_READ_ANGLE 0x92
#define LK_MOTOR_CMD_READ_STATE 0x9C
#define LK_MOTOR_CMD_TORQUE     0xA1
#define LK_MOTOR_CMD_SPEED      0xA2
#define LK_MOTOR_CMD_ANGLE      0xA4

#define LK_MOTOR_MAX_IQ         2000
#define LK_MOTOR_LOST_TICK      100

typedef struct LK_Motor_Born_Data_struct_t
{
    CAN_HandleTypeDef* hcan;

    uint8_t id;
} LK_Motor_Born_Data_t;

typedef struct LK_Motor_Rx_Data_struct_t
{
    int8_t temperature;
    int16_t current;
    float speed;
    float angle;
    uint16_t encoder;
    uint8_t error;
    uint32_t last_rx_tick;
    uint32_t last_angle_tick;
} LK_Motor_Rx_Data_t;

typedef struct LK_Motor_Tx_Data_struct_t
{
   int16_t torque;
   float speed;
   float angle;
   uint16_t max_speed;
} LK_Motor_Tx_Data_t;

typedef struct LK_Motor_struct_t
{
    LK_Motor_Born_Data_t* born_data;
    LK_Motor_Rx_Data_t* rx_data;
    LK_Motor_Tx_Data_t* tx_data;
    uint8_t tx_buf[8];
} LK_Motor_t;

void LK_Motor_Update(LK_Motor_t* motor, uint8_t* rxBuf);
void LK_Motor_Enable(LK_Motor_t* motor);
void LK_Motor_Stop(LK_Motor_t* motor);
void LK_Motor_Close(LK_Motor_t* motor);
void LK_Motor_Read_State(LK_Motor_t* motor);
void LK_Motor_Read_Angle(LK_Motor_t* motor);
void LK_Set_Torque(LK_Motor_t* motor);
void LK_Set_angle(LK_Motor_t* motor);
uint8_t LK_Motor_Is_Ready(LK_Motor_t* motor);
uint8_t LK_Motor_Angle_Ready(LK_Motor_t* motor);
void LK_Set_Speed(LK_Motor_t* motor);

#endif

