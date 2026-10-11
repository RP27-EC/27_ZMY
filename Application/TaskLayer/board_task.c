#include "board_task.h"
#include "board.h"

#define BOARD_LIFT_TX_TICK   5

void StartBoardTask(void const * argument)
{
	uint32_t tick = 0;

	for(;;)
	{
		Board_Tx_Gimbal_Data(&board);

		tick ++;
		if(tick % BOARD_LIFT_TX_TICK == 0)
		{
			Board_Tx_Lift_Data(&board);
		}

		osDelay(1);
	}
}
