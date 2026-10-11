/**
 * @file board_protocol.c
 * @brief 板间层，发送云台目标，发射启动，升降启动的指令
 * 
 */


#include "board_protocol.h"
#include "drv_can.h"

static uint16_t board_float_to_uint(float x, float x_min, float x_max, uint16_t max_u)
{
	if(x < x_min)
	{
		x = x_min;
	}
	else if(x > x_max)
	{
		x = x_max;
	}

	return (uint16_t)((x - x_min) * (float)max_u / (x_max - x_min));
}

static float board_uint_to_float(uint16_t x_int, float x_min, float x_max, uint16_t max_u)
{
	return (float)x_int * (x_max - x_min) / (float)max_u + x_min;
}

uint8_t Board_Is_Online(Board_t* board)
{
	if(board->last_rx_tick == 0)
	{
		return 0;
	}

	if(HAL_GetTick() - board->last_rx_tick > BOARD_LOST_TICK)
	{
		return 0;
	}

	return 1;
}

void Board_Tx_Car_Data(Board_t* board)
{
	uint8_t pkt[8] = {0};

	pkt[0] |= (board->tx_data->car_data.car_state & 0x03) << 0;

	CAN_SendData(&hfdcan2, ID_CAR_DATA, pkt);
}

void Board_Tx_Gimbal_Target(Board_t* board)
{
	uint8_t pkt[8] = {0};

	uint16_t t1 = board_float_to_uint(board->tx_data->gimbal_target_data.yaw_mec_tar, -12.5f, 12.5f, 65535);
	uint16_t t2 = board_float_to_uint(board->tx_data->gimbal_target_data.yaw_imu_tar, -360.f, 360.f, 65535);
	uint16_t t3 = board_float_to_uint(board->tx_data->gimbal_target_data.pitch_mec_tar, -4.f, 4.f, 65535);

	pkt[0] = t1 >> 8;
	pkt[1] = t1;
	pkt[2] = t2 >> 8;
	pkt[3] = t2;
	pkt[4] = t3 >> 8;
	pkt[5] = t3;
	pkt[6] = board->tx_data->gimbal_target_data.yaw_mode;

	CAN_SendData(&hfdcan2, ID_GIMBAL_TARGET, pkt);
}

void Board_Rx_Gimbal_Data(Board_t* board, uint8_t* rxBuf)
{
	uint16_t t1 = ((uint16_t)rxBuf[0] << 8) | rxBuf[1];
	uint16_t t2 = ((uint16_t)rxBuf[2] << 8) | rxBuf[3];
	uint16_t t3 = ((uint16_t)rxBuf[4] << 8) | rxBuf[5];
	uint16_t t4 = ((uint16_t)rxBuf[6] << 8) | rxBuf[7];

	board->rx_data->gimbal_data.yaw_mec   = board_uint_to_float(t1, -12.5f, 12.5f, 65535);
	board->rx_data->gimbal_data.pitch_mec = board_uint_to_float(t2, -12.5f, 12.5f, 65535);
	board->rx_data->gimbal_data.yaw_imu   = board_uint_to_float(t3, -360.f, 360.f, 65535);
	board->rx_data->gimbal_data.pitch_imu = board_uint_to_float(t4, -360.f, 360.f, 65535);

	board->last_rx_tick = HAL_GetTick();
}
void Board_Rx_Lift_Data(Board_t* board, uint8_t* rxBuf)
{
	board->rx_data->lift_state = rxBuf[0];

	board->last_rx_tick = HAL_GetTick();
}

void Board_Tx_Shoot_Data(Board_t* board)
{
	uint8_t pkt[8] = {0};

	pkt[0] |= board->tx_data->shoot_data.enable << 0;
	pkt[0] |= board->tx_data->shoot_data.mode  << 1;
	pkt[0] |= board->tx_data->shoot_data.trigger << 2;
	pkt[0] |= board->tx_data->shoot_data.lift << 3;

	CAN_SendData(&hfdcan2, ID_SHOOT_DATA, pkt);
}
