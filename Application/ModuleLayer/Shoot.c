#include "shoot.h"
#include "board.h"
#include "rc_sensor.h"

Shoot_t shoot;

static uint8_t shoot_wheel_latch = 0;
/**
 * @brief 发射下板控制，波轮在100-550的时候单发，波轮在550以上连发
 * 
 * @param shoot 
 */
//mode = 0是单发，mode = 1是连发。trigger是是否按下了 
static void Shoot_Target_Update(Shoot_t* shoot)
{
    if(HAL_GetTick() - rc_sensor_info.last_rx_tick > SHOOT_RC_LOST_TICK)
    {
        shoot_wheel_latch = 0;
        shoot->target.enable = 0;
        shoot->target.mode = 0;
        shoot->target.trigger = 0;
		return;
    }
    //只有在拨一下复位再拨才发一颗，与上板的逻辑配合
    if (rc_sensor_info.thumbwheel.value >= SHOOT_RC_TRIGGER_SIGNAL)
    {
        shoot_wheel_latch = 1;
    }
    else if (rc_sensor_info.thumbwheel.value < SHOOT_RC_TRIGGER_RELEASE)
    {
        shoot_wheel_latch = 0;
    }

    if (rc_sensor_info.s1.value == RC_SW_DOWN)
    {
        shoot->target.enable = 1;
    }
    else
    {
        shoot->target.enable = 0;
    }
    shoot->target.mode = 0;
    shoot->target.trigger = 0;

    if(rc_sensor_info.s1.value != RC_SW_DOWN)
    {
        return;
    }

    if(shoot_wheel_latch != 0)
    {
        if(rc_sensor_info.thumbwheel.value >= SHOOT_RC_TRIGGER_CONTINUE)
        {
            shoot->target.mode = 1;
        }
        else
        {
            shoot->target.mode = 0;
        }
        shoot->target.trigger = 1;
    }

    if(rc_sensor_info.mouse_btn_l.value != 0)
    {
        shoot->target.mode = 0;
        shoot->target.trigger = 1;
    }
    else if(rc_sensor_info.mouse_btn_r.value != 0)
    {
        shoot->target.mode = 1;
        shoot->target.trigger = 1;
    }
}

static void Shoot_Tx_Update(Shoot_t* shoot)
{
    board_tx_data.shoot_data.enable = shoot->target.enable;
    board_tx_data.shoot_data.mode = shoot->target.mode;
    board_tx_data.shoot_data.trigger = shoot->target.trigger;
}

void Shoot_Work(Shoot_t* shoot)
{
    Shoot_Target_Update(shoot);
    Shoot_Tx_Update(shoot);
}
