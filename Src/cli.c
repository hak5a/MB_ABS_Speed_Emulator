/*
 * cli.c
 *
 *  Created on: Feb 15, 2018
 *      Author: hak5a
 */

/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pulse_generator.h"

/* Private defines -----------------------------------------------------------*/

#define COMMAND_BUFFER_SIZE 20
#define COMMAND_SIZE 		10
#define ATTRIBUTE_SIZE		10

/* Private variables ---------------------------------------------------------*/

extern UART_HandleTypeDef huart1;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;


unsigned int uptime = 0;
uint32_t time_uptime = 0;

char rx_command_buffer[COMMAND_BUFFER_SIZE];
unsigned char rx_command_buffer_i = 0;

/* Private function prototypes -----------------------------------------------*/

void CLI_Parse_Commands(char *command);



void CLI_run(void)
{
	char rx_c;

	if( HAL_UART_Receive (&huart1, (uint8_t *)&rx_c, 1, 10) == HAL_OK )
	{
	  if(rx_command_buffer_i < COMMAND_BUFFER_SIZE)
	  {
		  rx_command_buffer[rx_command_buffer_i++] = rx_c;
		  if(rx_c == '\n')
		  {
			  rx_command_buffer[rx_command_buffer_i] = 0;
			  CLI_Parse_Commands(rx_command_buffer);
			  rx_command_buffer_i = 0;
		  }
	  }
	  else
	  {
		  rx_command_buffer_i = 0;
	  }
	}

	uint32_t time_now = HAL_GetTick();
	if( (time_now - time_uptime) >= 10000 )
	{
	  time_uptime = time_now;
	  printf("Uptime: %u sec\n\r", 10 * ++uptime);
	}
}

void CLI_Parse_Commands(char *full_command)
{
	//printf("debug... Parse command... %s \n\n\ŋ\r", full_command);

	char temp_str[COMMAND_BUFFER_SIZE];
	char parsed_command[COMMAND_SIZE];
	char parsed_attribute[ATTRIBUTE_SIZE];

    strcpy(temp_str, full_command);
    char *token;

    /* get the first token */
    token = strtok(temp_str, " \n");
    if( token != NULL )
        strcpy(parsed_command, token);
    else
        strcpy(parsed_command, " ");
    /* get the second token */
    token = strtok(NULL, " \n");
    if( token != NULL )
        strcpy(parsed_attribute, token);
    else
        strcpy(parsed_attribute, " ");

    //printf("debug... parsed command  : %s\n\r", parsed_command);
    //printf("debug... parsed attribute: %s\n\r", parsed_attribute);
    float temp_float;

    if(strcmp(parsed_command, "hello") == 0)
    {
        printf("Hello %s!\n\r", parsed_attribute);
    }
    else if(strcmp(parsed_command, "speed_fl") == 0)
    {
        temp_float = atof(parsed_attribute);
        printf("OK, Front Left speed is now %.1f km/h\n\r", temp_float);
        pulse_generator_set_speed_FL(temp_float);
    }
    else if(strcmp(parsed_command, "speed_fr") == 0)
    {
        temp_float = atof(parsed_attribute);
        printf("OK, Front Right speed is now %.1f km/h\n\r", temp_float);
        pulse_generator_set_speed_FR(temp_float);
    }
    else if(strcmp(parsed_command, "speed_diff") == 0)
    {
        temp_float = atof(parsed_attribute);
        printf("OK, Diff speed is now %.1f km/h\n\r", temp_float);
        pulse_generator_set_speed_DIFF(temp_float);
    }
    else
    {
        printf("\nSupported Commands:\n\r");
        printf("  speed_fl [speed]\n\r");
        printf("  speed_fr [speed]\n\r");
        printf("  speed_diff [speed]\n\r");
        printf("\n\r");
    }
    printf("\n\r");
}
