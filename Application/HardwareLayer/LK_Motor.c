#include "LK_Motor.h"
#include "string.h"

static void LK_Send_Command(LK_Motor_t *motor, uint8_t cmd)
{
    memset(motor->tx_buf, 0, 8);
    motor->tx_buf[0] = cmd;
    CAN_SendData(motor->born_data->hcan, LK_MOTOR_STDID + motor->born_data->id, motor->tx_buf);
}

void LK_Motor_Update(LK_Motor_t* motor, uint8_t* rxBuf)
{
    LK_Motor_Rx_Data_t* motor_rx_data = motor->rx_data;
    int64_t sum=0;
    switch(rxBuf[0])
    {
        case LK_MOTOR_CMD_READ_STATE:
        case LK_MOTOR_CMD_TORQUE:
        case LK_MOTOR_CMD_SPEED:
        case LK_MOTOR_CMD_ANGLE:
            motor_rx_data->temperature = (int8_t)rxBuf[1];
            motor_rx_data->current = (int16_t)((uint16_t)rxBuf[2]  | (uint16_t) rxBuf[3] << 8);
            motor_rx_data->speed = (float)((int16_t)((uint16_t)rxBuf[4] |( (uint16_t)rxBuf[5] << 8)));
            motor_rx_data->encoder = (uint16_t)((uint16_t)rxBuf[6] | (uint16_t)rxBuf[7] << 8);
            break;
        case LK_MOTOR_CMD_READ_ANGLE:
            for(int i=1;i<=7;i++)
            {
                sum |= ((int64_t)rxBuf[i] << (8*(i-1)));
            }
            sum = (int64_t)((uint64_t)sum << 8) >> 8;
            motor_rx_data->angle = (float)sum / 100.f;
            motor_rx_data->last_angle_tick = HAL_GetTick();
            break;
        case LK_MOTOR_CMD_READ_ENC:
            motor_rx_data->encoder = (uint16_t)((uint16_t)rxBuf[2] | (uint16_t)rxBuf[3] << 8);
            break;
        default:
            break;
    }
    motor_rx_data->last_rx_tick = HAL_GetTick();
}

void LK_Motor_Enable(LK_Motor_t* motor)
{
    LK_Send_Command(motor, LK_MOTOR_CMD_ENABLE);
}

void LK_Motor_Stop(LK_Motor_t* motor)
{
    LK_Send_Command(motor, LK_MOTOR_CMD_STOP);
}

void LK_Motor_Close(LK_Motor_t* motor)
{
    LK_Send_Command(motor, LK_MOTOR_CMD_CLOSE);
}

void LK_Motor_Read_State(LK_Motor_t* motor)
{
    LK_Send_Command(motor, LK_MOTOR_CMD_READ_STATE);
}

void LK_Motor_Read_Angle(LK_Motor_t* motor)
{
    LK_Send_Command(motor, LK_MOTOR_CMD_READ_ANGLE);
}

void LK_Set_Torque(LK_Motor_t* motor)
{
    int16_t iq = motor->tx_data->torque;
    if(iq > LK_MOTOR_MAX_IQ)
    {
        iq = LK_MOTOR_MAX_IQ;
    }
    else if(iq  < -LK_MOTOR_MAX_IQ)
    {
        iq = -LK_MOTOR_MAX_IQ;
    }
    motor->tx_data->torque = iq;

    memset(motor->tx_buf, 0, 8);
    motor->tx_buf[0] = LK_MOTOR_CMD_TORQUE;
    motor->tx_buf[4] = (uint8_t)(iq & 0xFF);
    motor->tx_buf[5] = (uint8_t)((iq >> 8) & 0xFF);

    CAN_SendData(motor->born_data->hcan, LK_MOTOR_STDID + motor->born_data->id, motor->tx_buf);
}

void LK_Set_angle(LK_Motor_t* motor)
{
    int32_t angle = (int32_t)(motor->tx_data->angle * 100.0f);
    uint32_t raw = (uint32_t)angle;
    uint16_t speed = motor->tx_data->max_speed;

    memset(motor->tx_buf, 0, 8);
    motor->tx_buf[0] = LK_MOTOR_CMD_ANGLE;
    motor->tx_buf[2] = (uint8_t)(speed & 0xFF);
    motor->tx_buf[3] = (uint8_t)((speed >> 8) & 0xFF);
    motor->tx_buf[4] = (uint8_t)(raw & 0xFF);
    motor->tx_buf[5] = (uint8_t)((raw >> 8) & 0xFF);
    motor->tx_buf[6] = (uint8_t)((raw >> 16) & 0xFF);
    motor->tx_buf[7] = (uint8_t)((raw >> 24) & 0xFF);
    CAN_SendData(motor->born_data->hcan, LK_MOTOR_STDID + motor->born_data->id, motor->tx_buf);

}

uint8_t LK_Motor_Is_Ready(LK_Motor_t* motor)
{
    uint32_t tick = HAL_GetTick();
    if(tick - motor->rx_data->last_rx_tick < LK_MOTOR_LOST_TICK)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

uint8_t LK_Motor_Angle_Ready(LK_Motor_t* motor)
{
    uint32_t tick = HAL_GetTick();
    if(motor->rx_data->last_angle_tick == 0)
    {
        return 0;
    }
    if(tick - motor->rx_data->last_angle_tick < LK_MOTOR_LOST_TICK)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

void LK_Set_Speed(LK_Motor_t* motor)
{
    int32_t speed = (int32_t)(motor->tx_data->speed * 100.f);
    uint32_t raw = (uint32_t)speed;

    memset(motor->tx_buf,0,8);
     motor->tx_buf[0] = LK_MOTOR_CMD_SPEED;
      motor->tx_buf[4] = (uint8_t)(raw & 0xFF);
      motor->tx_buf[5] = (uint8_t)((raw >> 8) & 0xFF);
      motor->tx_buf[6] = (uint8_t)((raw >> 16) & 0xFF);
      motor->tx_buf[7] = (uint8_t)((raw >> 24) & 0xFF);
      CAN_SendData(motor->born_data->hcan, LK_MOTOR_STDID + motor->born_data->id, motor->tx_buf);
}
