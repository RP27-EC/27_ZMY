#include "Fric.h"

static void Fric_Target_Update(Fric_t *fric);
static void Fric_Torque_Zero(Fric_t *fric);
static void Fric_Speed_Pid(Fric_t* fric);


float fric_Motor_Sign[FRIC_CNT] = {1.f, -1.f};
Fric_t Fric;

static float fric_speed_filter[FRIC_CNT];

static void Fric_Speed_Filter_Update(void)
{
    for (uint8_t i = 0; i < FRIC_CNT; i++)
    {
        fric_speed_filter[i] = Lowpass(fric_speed_filter[i], RM_Group.motor[i]->rx_data->encoder_speed, FRIC_SPEED_FILTER_K);
    }
}

static void Fric_Target_Update(Fric_t *fric)
{
    float want = (fric->target.enable != 0) ? FRIC_TARGET_SPEED : 0.f;
    fric->target.speed = RampFloat(want, fric->target.speed, FRIC_SPEED_RAMP);
}


static void Fric_Torque_Zero(Fric_t* fric)
{
    for (int i = 0; i < FRIC_CNT; i++)
    {
        RM_Group.motor[i]->tx_data->torque = 0.f;
        RM_Group.motor[i]->tx_data->torque_current = 0.f;
        RM_Group.motor[i]->tx_data->torque_current_raw = 0;
        fric->out.Out[i] = 0.f;
    }
    RM_Set_Torque(&RM_Group);
}

static void Fric_Speed_Pid(Fric_t* fric)
{
    for(uint8_t i = 0; i < FRIC_CNT; i++)
    {
        pid_ctrl_t* pid = RM_Group.motor[i]->ctrl_data->speed_ctrl;

        pid->target = fric->target.speed * fric_Motor_Sign[i];
        pid->measure = RM_Group.motor[i]->rx_data->encoder_speed;
        pid->err = pid->target - pid->measure;

        single_pid_ctrl(pid);

        fric->out.Out[i] = pid->out;
    }
}

static void Fric_Cmd_Transmit(Fric_t* fric)
{
    for(uint8_t i = 0; i < FRIC_CNT; i++)
    {
        RM_Group.motor[i]->tx_data->torque = fric->out.Out[i] * FRIC_RAW_TO_TORQUE;
    }
    RM_Set_Torque(&RM_Group);
}

void Fric_Work(Fric_t *fric)
{
    Fric_Target_Update(fric);
    Fric_Speed_Filter_Update();

    if (Fric_Is_Online() == 0 || fric->target.speed <= 0.f)
    {
        Fric_Torque_Zero(fric);
        return;
    }

    Fric_Speed_Pid(fric);

    Fric_Cmd_Transmit(fric);
}

uint8_t Fric_Is_Online(void)
{
    for(uint8_t i = 0; i < FRIC_CNT; i++)
    {
        if(HAL_GetTick() - RM_Group.motor[i]->rx_data->last_rx_tick > FRIC_LOST_TICK)
        {
            return 0;
        }
    }
    return 1;
}

uint8_t Fric_Is_Ready(void)
{
    uint8_t i = 0; 
    for(;i < FRIC_CNT;i++)
    {
        if(abs(fric_speed_filter[i]) < FRIC_READY_SPEED)
        {
            return 0;
        }

    }
    return 1;
}
