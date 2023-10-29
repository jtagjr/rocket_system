/*
 * debug_tx_uart.c
 *
 *  Created on: Oct 15, 2021
 *      Author: johnt
 */
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "semphr.h"

#ifdef DEBUG
enum { MAX_LENGTH = 5000 };

extern UART_HandleTypeDef huart3;
static uint32_t aligned_buffer[MAX_LENGTH/4];
static char* buffer = (char*)aligned_buffer;
static uint16_t tail = 0;
static uint16_t head = 0;
static uint16_t length_to_transfer = 0;
static SemaphoreHandle_t printf_mutex;
static uint16_t highwaterMark = 0;

void initialize_debug_printf()
{
	printf_mutex = xSemaphoreCreateMutex();
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
int __io_putchar(int ch)
{
  buffer[tail++] = (char)ch;
  if (tail == MAX_LENGTH)
  {
	  tail = 0 ;
  }
  return ch;
}

int _write(int fd, char * ptr, int len)
{
  for(int i=0; i<len; ++i)
  {
	  __io_putchar(*(ptr+i));
  }
  return len;
}

void calculate_high_water_mark()
{
	uint16_t length = tail - head;
	if (head > tail)
	{
		length = MAX_LENGTH - head + tail;
	}
	if (highwaterMark < length)
	{
		highwaterMark = length;
	}
}

uint16_t calculate_length()
{
	uint16_t length = tail - head;
	if (head > tail)
	{
		length = MAX_LENGTH - head;
	}
	return length;
}

void thread_safe_printf(const char *format, ...)
{
	// Uses mutex if a higher priority task is pending on this mutex the lower priority task that has it will get it's priority elevated.
	xSemaphoreTake(printf_mutex, (TickType_t) 150); // Wait for max 150ms this is the amount of time it'll take for all 2000 bytes to be written out by DMA
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
    fflush(stdout);
    calculate_high_water_mark();
    xSemaphoreGive(printf_mutex);
}

void thread_safe_printf_newline(const char *format, ...)
{
	// Uses mutex if a higher priority task is pending on this mutex the lower priority task that has it will get it's priority elevated.
	xSemaphoreTake(printf_mutex, (TickType_t) 150); // Wait for max 150ms this is the amount of time it'll take for all 2000 bytes to be written out by DMA
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
    fflush(stdout);
    __io_putchar('\r');
    __io_putchar('\n');
    calculate_high_water_mark();
    xSemaphoreGive(printf_mutex);
}

uint16_t DebugHighwaterMark()
{
	return highwaterMark;
}

void TransmitData(uint32_t timeout)
{
	head += length_to_transfer;
	if (head == MAX_LENGTH)
	{
		head = 0;
	}

	length_to_transfer = calculate_length();

	if (length_to_transfer)
	{
		HAL_UART_Transmit(&huart3,  (uint8_t*)&buffer[head], length_to_transfer, timeout);
	}
}
#endif


