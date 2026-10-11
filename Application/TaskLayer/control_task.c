#include "control_task.h"
#include "gimbal.h"
#include "Lift.h"
#include "Fric.h"
#include "Dial.h"
#include "Shoot.h"

float pitch;
float roll;
float yaw;

void StartControlTask(void const * argument)
{
	for(;;)
	{
		imu_sensor.update(&imu_sensor);
		pitch = imu_sensor.info->base_info.pitch;
		roll = imu_sensor.info->base_info.roll;
		yaw = imu_sensor.info->base_info.yaw;

		Gimbal_Work(&gimbal);
		Lift_Work(&lift);
		Fric_Work(&Fric);
		Dial_Work(&dial);
		Shoot_Work(&shoot);
		osDelay(1);
	}
}
