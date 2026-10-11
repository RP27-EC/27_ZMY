#include "Shoot.h"
#include "board.h"
#include "Fric.h"
#include "Dial.h"

Shoot_t shoot;

/**
 * @brief 这只有将下板目标数据搬到结构体的作用，拨盘和摩擦轮有自己的工作函数
 * 
 * @param shoot 
 */
static void Shoot_Check(Shoot_t* shoot)
{
    if(Board_Is_Online(&board) == 0  || board.rx_data->car_data.car_state == CAR_SLEEP)
	{
        shoot->target.enable = 0;
        shoot->target.mode = 0;
        shoot->target.trigger = 0;
		return;
	}

    shoot->target.enable = board.rx_data->shoot_target.enable;
    shoot->target.mode = board.rx_data->shoot_target.mode;
    shoot->target.trigger = board.rx_data->shoot_target.trigger;
}

static void Shoot_Cmd_Transmit(Shoot_t* shoot)
{
    Fric.target.enable = shoot->target.enable;
    dial.target.enable = shoot->target.enable;
    dial.target.mode = shoot->target.mode;
    dial.target.trigger = shoot->target.trigger;
}

void Shoot_Work(Shoot_t* shoot)
{
    Shoot_Check(shoot);
    Shoot_Cmd_Transmit(shoot);  
}
