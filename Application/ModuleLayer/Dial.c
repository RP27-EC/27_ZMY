#include "Dial.h"

Dial_t dial;

static uint32_t dial_read_tick = 0;

static void Dial_Feedback_Request(Dial_t* dial)
{
    dial_read_tick++;

    if(dial_read_tick % DIAL_READ_ANGLE_TICK == 0) //读角度
    {
        LK_Motor_Read_Angle(&Dial_Motor);
    }

    if(dial_read_tick % DIAL_READ_STATE_TICK == 0) //读转速和电流
    {
        LK_Motor_Read_State(&Dial_Motor);
    }

    if(dial->state != DIAL_SLEEP && dial_read_tick % DIAL_ENABLE_TICK == 0) //使能帧
    {
        LK_Motor_Enable(&Dial_Motor);
    }
}
//搬数据进来，判断摩擦轮转速有没达到阈值还有检测
static void Dial_Extern_Update(Dial_t* dial)
{
    dial->rx_data.angle = Dial_Motor.rx_data->angle;
    dial->rx_data.speed = Dial_Motor.rx_data->speed;
    dial->rx_data.current = Dial_Motor.rx_data->current;
    dial->rx_data.online = LK_Motor_Is_Ready(&Dial_Motor);

    if (Fric_Is_Online() != 0 && Fric_Is_Ready() != 0)
    {
        dial->fric_lost_tick = 0;
        dial->rx_data.fric_online = 1;
    }
    else if (dial->fric_lost_tick < DIAL_FRIC_LOST_TICK)
    {
        dial->fric_lost_tick++;
    }
    else
    {
        dial->rx_data.fric_online = 0;
    }

    if (dial->rx_data.online == 0)
    {
        dial->rx_data.fric_online = 0;
    }
}

static uint8_t Dial_Block_Check(Dial_t *dial)
{
    //如果转角度大于堵转设定角度就不认为堵转
    if(abs(dial->rx_data.angle - dial->stuck_angle) >= DIAL_STUCK_ANGLE)
    {
        dial->stuck_angle = dial->rx_data.angle;
        dial->stuck_tick = 0;
    }
    //如果速度不高于堵转设定最低速度，但有大电流时判断为堵转
    if(abs(dial->rx_data.speed) < DIAL_STUCK_SPEED && dial->rx_data.current > DIAL_STUCK_CURRENT)
    {
        dial->stuck_tick++;
        if(dial->stuck_tick >= DIAL_STUCK_TICK)
        {
            return 1;
        }
    }
    else
    {
        dial->stuck_tick = 0;
    }

    return 0;
}
/**
 * @brief 这个状态机简单来说就是，进入然后看是否有单发排队（因为每次单发需要间隔一点点时间），然后堵转同时判断，在之后看有无连发请求，有则进入连发（并且同时堵转判断），
 *        如果有堵转则进入到reload状态（回一个角度），堵转和连发之后都进入stop状态，最后停下，不断电，一般都停在waitting状态
 * 
 * @param dial 
 */
static void Dial_State_Machine(Dial_t * dial)
{
    float oneshoot = DIAL_ONESHOT_ANGLE;
    float current_angle = dial->rx_data.angle;
    uint8_t stop_req = 0;

    if (dial->target.enable == 0
        || (dial->rx_data.fric_online == 0 && dial->state != DIAL_SINGLE))
    {
        stop_req = 1;
        dial->target.trigger = 0;
        dial->target.mode = 0;
    }
    else if (dial->state == DIAL_CONTINUOUS && dial->target.trigger == 0)
    {
        stop_req = 1;
    }

    if (stop_req != 0)
    {
        dial->rearm_tick = 0;
        dial->single_armed = 0;
    }
    else if (dial->target.trigger == 0)
    {
        if (dial->rearm_tick < DIAL_REARM_TICK)
        {
            dial->rearm_tick++;
        }
        else
        {
            dial->single_armed = 1;
        }
    }
    else
    {
        dial->rearm_tick = 0;
    }

    if (dial->target.mode == 0 && dial->target.trigger != 0 && dial->single_armed != 0)
    {
        dial->single_armed = 0;
        dial->rearm_tick = 0;
        dial->single_pending = 1;
        dial->pending_tick = DIAL_PENDING_TICK;
    }

    if (dial->single_pending != 0)
    {
        if (dial->pending_tick > 0)
        {
            dial->pending_tick--;
        }
        else
        {
            dial->single_pending = 0;
        }
    }

    if (stop_req != 0 && dial->state != DIAL_SLEEP && dial->state != DIAL_STOP)
    {
        dial->out.speed = dial->rx_data.speed;
        dial->state = DIAL_STOP;
        dial->state_tick = 0;
        dial->stuck_tick = 0;
    }

    dial->state_tick++;
    switch (dial->state)
    {
    case DIAL_SLEEP:
        dial->out.angle = current_angle;
        dial->out.torque = 0;
        dial->out.speed = 0;
        dial->stuck_tick = 0;

        if (stop_req == 0)
        {
            dial->state = DIAL_WAITING;
            dial->state_tick = 0;
            dial->sync_flag = 1;
            LK_Motor_Enable(&Dial_Motor);
        }
        break;
    case DIAL_STOP:
        dial->out.speed = RampFloat(0.f, dial->out.speed, DIAL_STOP_RAMP);

        if (abs(dial->out.speed) <= 1.f || dial->state_tick >= DIAL_STOP_TIMEOUT)
        {
            dial->out.speed = 0.f;
            dial->out.torque = 0.f;
            dial->state = DIAL_SLEEP;
            dial->state_tick = 0;
            dial->sync_flag = 1;
        }
        break;
    case DIAL_WAITING:
        if (dial->sync_flag != 0)
        {
            dial->out.angle = current_angle;
            dial->sync_flag = 0;
        }
        dial->state_tick = 0;
        dial->stuck_tick = 0;
        if (dial->single_pending != 0)
        {
            dial->single_pending = 0;
            dial->out.angle += oneshoot;
            dial->state = DIAL_SINGLE;
            dial->stuck_tick = 0;
            dial->state_tick = 0;
            dial->single_cnt++;
            dial->single_exit_tick = 0;
        }
        else if (dial->target.mode == 1 && dial->target.trigger != 0)
        {
            dial->out.speed = DIAL_FIRE_HZ * DIAL_SPEED_PER_HZ;
            dial->state = DIAL_CONTINUOUS;
            dial->state_tick = 0;
            dial->stuck_tick = 0;
        }
        break;
    case DIAL_SINGLE:
        if(Dial_Block_Check(dial) != 0)
        {
            dial->block_cnt++;
            dial->single_exit_tick = dial->state_tick;
            dial->out.angle = current_angle - oneshoot;
            dial->state = DIAL_REVERSE;
            dial->state_tick = 0;
            dial->stuck_tick = 0;
            break;
        }
        if(dial->state_tick >= DIAL_SINGLE_TIMEOUT
            || (abs(dial->out.angle - current_angle) <= DIAL_ARRIVE_ERROR
                && abs(dial->rx_data.speed) <= DIAL_SETTLE_SPEED)
            || (abs(dial->out.angle - current_angle) <= DIAL_ARRIVE_LOOSE
                && abs(dial->rx_data.speed) <= DIAL_STUCK_SPEED))
        {
            dial->single_exit_tick = dial->state_tick;
            dial->state = DIAL_WAITING;
            dial->state_tick = 0;
            dial->stuck_tick = 0;
        }
        break;
    case DIAL_CONTINUOUS:
        dial->out.speed = DIAL_FIRE_HZ * DIAL_SPEED_PER_HZ;
        if (Dial_Block_Check(dial) != 0)
        {
            dial->block_cnt++;
            dial->out.angle = current_angle - oneshoot;
            dial->state = DIAL_REVERSE;
            dial->state_tick = 0;
            dial->stuck_tick = 0;
            break;
        }
        break;
    case DIAL_REVERSE:
        if(abs(dial->out.angle - current_angle) <= DIAL_ARRIVE_ERROR ||Dial_Block_Check(dial) != 0 ||dial->state_tick >= DIAL_REVERSE_TIMEOUT)
        {
            dial->out.angle = current_angle + oneshoot;
            dial->state = DIAL_RELOAD;
            dial->state_tick = 0;
            dial->stuck_tick = 0;
        }
        break;
    case DIAL_RELOAD:
        if(abs(dial->out.angle - current_angle) <= DIAL_ARRIVE_ERROR || Dial_Block_Check(dial) != 0 || dial->state_tick >= DIAL_RELOAD_TIMEOUT)
        {
            dial->state = DIAL_WAITING;
            dial->state_tick =0;
            dial->stuck_tick = 0;

        }
        break;
    default:
    dial->state = DIAL_SLEEP;
        break;
    }
}
//把状态机输出的状态都进行相应输出
static void Dial_Output(Dial_t *dial)
{
    if(dial->state == DIAL_SLEEP)
    {
        Dial_Motor.tx_data->torque = dial->out.torque;
        LK_Set_Torque(&Dial_Motor);
        return;
    }
    if(dial->state == DIAL_CONTINUOUS || dial->state == DIAL_STOP)
    {
        Dial_Motor.tx_data->speed =dial->out.speed;
        LK_Set_Speed(&Dial_Motor);
        return;
    }
    Dial_Motor.tx_data->angle = dial->out.angle;
    Dial_Motor.tx_data->max_speed = DIAL_MAX_SPEED;
    LK_Set_angle(&Dial_Motor);
}

void Dial_Work(Dial_t* dial)
{
    Dial_Feedback_Request(dial);
    Dial_Extern_Update(dial);
    Dial_State_Machine(dial);
    Dial_Output(dial);
}
