/*
 * pulse_generator.c
 *
 *  Created on: Apr 4, 2018
 *      Author: hak5a
 */

#include "main.h"
#include "stm32f1xx_hal.h"
#include "settings.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

extern struct abs_channel abs_channel_FL;
extern struct abs_channel abs_channel_FR;
extern struct abs_channel abs_channel_DIFF;


void pulse_generator_set_speed_FL(float speed)
{
	  uint32_t f_Sys_Clock = HAL_RCC_GetSysClockFreq();
	  uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim1);

	  float f = speed * abs_channel_FL.abs_pulse_ratio;
	  uint32_t ps = (uint32_t)( (float)f_Sys_Clock / (float)(period+1) / f );
	  __HAL_TIM_SET_PRESCALER(&htim1, ps);
}

void pulse_generator_set_speed_FR(float speed)
{
	  uint32_t f_Sys_Clock = HAL_RCC_GetSysClockFreq();
	  uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim2);

	  float f = speed * abs_channel_FR.abs_pulse_ratio;
	  uint32_t ps = (uint32_t)( (float)f_Sys_Clock / (float)(period+1) / f );
	  __HAL_TIM_SET_PRESCALER(&htim2, ps);
}

void pulse_generator_set_speed_DIFF(float speed)
{
	  uint32_t f_Sys_Clock = HAL_RCC_GetSysClockFreq();
	  uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim3);

	  float f = speed * abs_channel_DIFF.abs_pulse_ratio;
	  uint32_t ps = (uint32_t)( (float)f_Sys_Clock / (float)(period+1) / f );
	  __HAL_TIM_SET_PRESCALER(&htim3, ps);
}

