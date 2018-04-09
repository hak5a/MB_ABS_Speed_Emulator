/*
 * pulse_generator.c
 *
 *  Created on: Apr 4, 2018
 *      Author: hak5a
 */

#include "main.h"
#include "stm32f1xx_hal.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;


void pulse_generator_set_speed_FL(float speed)
{
	  uint32_t f_Sys_Clock = HAL_RCC_GetSysClockFreq();
	  uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim1);

	  float f = speed * ABS_PULSE_RATIO_FL;
	  uint32_t ps = (uint32_t)( (float)f_Sys_Clock / (float)period / f );
	  __HAL_TIM_SET_PRESCALER(&htim1, ps);
}

void pulse_generator_set_speed_FR(float speed)
{
	  uint32_t f_Sys_Clock = HAL_RCC_GetSysClockFreq();
	  uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim2);

	  float f = speed * ABS_PULSE_RATIO_FR;
	  uint32_t ps = (uint32_t)( (float)f_Sys_Clock / (float)period / f );
	  __HAL_TIM_SET_PRESCALER(&htim2, ps);
}

void pulse_generator_set_speed_DIFF(float speed)
{
	  uint32_t f_Sys_Clock = HAL_RCC_GetSysClockFreq();
	  uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim3);

	  float f = speed * ABS_PULSE_RATIO_DIFF;
	  uint32_t ps = (uint32_t)( (float)f_Sys_Clock / (float)period / f );
	  __HAL_TIM_SET_PRESCALER(&htim3, ps);
}

