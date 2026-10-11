
/**
 * @file        rp_math.c
 * @author      RobotPilots
 * @Version     v1.1
 * @brief       RobotPilots Robots' Math Libaray.
 * @update
 *              v1.0(11-September-2020)
 *              v1.1(13-November-2021)
 *                  1.增加位操作函数
 */

/* Includes ------------------------------------------------------------------*/
#include "rp_math.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
 * @brief  低通滤波,K∈(0,1)，K 越大，滤波效果越弱
 * @param  上次的滤波输出X_last  ，新的输入X_new ，滤波系数K
 * @return  滤波后数值
 */
float Lowpass(float X_last, float X_new, float K)
{
	return (X_last + (X_new - X_last) * K);
}


float motor_half_cycle(float angle, float max)
{
	if (abs(angle) > (max / 2))
	{
		if (angle >= 0)
			angle += -max;
		else
			angle += max;
	}
	return angle;
}

int16_t RampInt(int16_t final, int16_t now, int16_t ramp)
{
	int32_t buffer = 0;

	buffer = final - now;
	if (buffer > 0)
	{
		if (buffer > ramp)
			now += ramp;
		else
			now += buffer;
	}
	else
	{
		if (buffer < -ramp)
			now += -ramp;
		else
			now += buffer;
	}

	return now;
}

float RampFloat(float final, float now, float ramp)
{
	float buffer = 0;

	buffer = final - now;
	if (buffer > 0)
	{
		if (buffer > ramp)
			now += ramp;
		else
			now += buffer;
	}
	else
	{
		if (buffer < -ramp)
			now += -ramp;
		else
			now += buffer;
	}

	return now;
}




float DeathZoom(float input, float center, float death)
{
	if (abs(input - center) < death)
		return center;
	return input;
}

uint16_t float_to_uint(float x, float x_min, float x_max, uint16_t max_u)
{
    if (x < x_min)
    {
        x = x_min;
    }
    else if (x > x_max)
    {
        x = x_max;
    }

    return (uint16_t)((x - x_min) * (float)max_u / (x_max - x_min));
}

float uint_to_float(uint16_t u, float x_min, float x_max, uint16_t max_u)
{
    return (float)u * (x_max - x_min) / (float)max_u + x_min;
}


