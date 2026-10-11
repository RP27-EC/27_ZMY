#include "can_protocol.h"
#include "board.h"
#include "motor.h"


void CAN1_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	switch (rxId)
	{
		case ID_R_FRIC:
		{
			RM_Motor_Update(&R_Fric, rxBuf);
			break;
		}
		case ID_L_FRIC:
		{
			RM_Motor_Update(&L_Fric, rxBuf);
			break;
		}
		case ID_GIMB_P:
		{
			DM_Motor_Update(&Pitch_Motor, rxBuf);
			break;
		}
		case ID_DIAL:
		{
			LK_Motor_Update(&Dial_Motor, rxBuf);
			break;
		}
		case ID_LIFT:
		{
			RM_Motor_Update(&Lift_Motor, rxBuf);
			break;
		}

		default:
			break;
	}
}

void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{

	switch (canId)
	{
		case ID_CAR_DATA:
		{
			Board_Rx_Car_Data(&board, rxBuf);
			break;
		}
		case ID_GIMBAL_TARGET:
		{
			Board_Rx_Gimbal_Target(&board, rxBuf);
			break;
		}
		case ID_GIMB_Y:
		{
			DM_Motor_Update(&Yaw_Motor, rxBuf);
			break;
		}
		case ID_SHOOT_TARGET:
		{
			Board_Rx_Shoot_Target(&board,rxBuf);
			break;
		}
		default:
			break;
	}
}
