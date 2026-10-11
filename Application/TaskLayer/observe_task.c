#include "observe_task.h"

extern osSemaphoreId_t semTaskObserveToCtrl;
extern osSemaphoreId_t semTaskCtrlToObserve;

void StartUpdataTask(void const * argument)
{
	for(;;)
	{
		
		
		if(imu_sensor.work_state.err_code == IMU_NONE_ERR ||
			imu_sensor.work_state.err_code == IMU_DATA_CALI)
		{
			imu_sensor.update(&imu_sensor);
		}
		

		osSemaphoreRelease(semTaskObserveToCtrl);
		osSemaphoreAcquire(semTaskCtrlToObserve, osWaitForever); 

		 osDelay(1);
	}
}
