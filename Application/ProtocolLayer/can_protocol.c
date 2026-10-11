#include "can_protocol.h"
#include "board.h"


void CAN1_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	switch (rxId)
	{
		case ID_WHEEL_LF:
			RM_Motor_Update(&wheel_motor[WHEEL_LF], rxBuf);
			break;
		
		case ID_WHEEL_LB:
			RM_Motor_Update(&wheel_motor[WHEEL_LB], rxBuf);
			break;
		
		case ID_WHEEL_RF:
			RM_Motor_Update(&wheel_motor[WHEEL_RF], rxBuf);
			break;
		
		case ID_WHEEL_RB:
			RM_Motor_Update(&wheel_motor[WHEEL_RB], rxBuf);
			break;

		default:
			break;
	}
}


void CAN2_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	switch (rxId)
	{
		case ID_GIMBAL_DATA:
			Board_Rx_Gimbal_Data(&board, rxBuf);
			break;

		case ID_LIFT_DATA:
			Board_Rx_Lift_Data(&board, rxBuf);
			break;

		default:
			break;
	}
}

