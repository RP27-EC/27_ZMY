#include "control_task.h"
#include "chassis.h"
#include "gimbal.h"
#include "Lift.h"
#include "Shoot.h"

void StartCtrlTask(void const * argument)
{

	for(;;)
	{
		Chassis_Work(&chassis);
		Lift_Work(&lift);
		Gimbal_Work(&gimbal);
		Shoot_Work(&shoot);
		osDelay(1);
	}
}
