/*
 * cli.c
 *
 *  Created on: Feb 15, 2018
 *      Author: hak5a
 */

/* Includes ------------------------------------------------------------------*/

#include <stdio.h>
#include "main.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <string.h>

/* Private defines -----------------------------------------------------------*/

#define COMMAND_BUFFER_SIZE 20
#define COMMAND_SIZE 		10
#define ATTRIBUTE_SIZE		10

/* Private variables ---------------------------------------------------------*/

extern UART_HandleTypeDef huart1;

unsigned int uptime = 0;
uint32_t time_uptime = 0;

uint8_t rx_command_buffer[COMMAND_BUFFER_SIZE];
uint8_t rx_command_buffer_i = 0;

/* Private function prototypes -----------------------------------------------*/

void CLI_Parse_Commands(uint8_t *command);



void CLI_run(void)
{
	uint8_t rx_c;

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

void CLI_Parse_Commands(uint8_t *full_command)
{
	printf("Parse command... %s \n\r", full_command);

	char parsed_command[COMMAND_SIZE] = " ";
	char parsed_attribute[ATTRIBUTE_SIZE] = " ";

	size_t place_of_space = strcspn(full_command, " ");
	printf("debug... place of space: %u\n\r", (unsigned int)place_of_space);

	strncpy(parsed_command, full_command, place_of_space);
    *(parsed_command+place_of_space) = NULL;
	printf("debug... parsed command: %s\n\r", parsed_command);

	printf("debug full command size: %u\n\r", strlen(full_command));
    if(strlen(full_command) > place_of_space)
    {
        strcpy(parsed_attribute, full_command+place_of_space+1);
        size_t place_of_end = strcspn(parsed_attribute, "\n");
        printf("debug... place of end: %u\n\r", (unsigned int)place_of_end);
        *(parsed_attribute+place_of_end) = NULL;
    }
    printf("debug parsed attribute: %s\n\r", parsed_attribute);




}
