/*
 * sa818s.h
 *
 *  Created on: Feb 13, 2026
 *      Author: HP
 */

#ifndef SA818S_H_
#define SA818S_H_

#include "stm32f4xx_hal.h" // Adjust for your STM32 series
#include <string.h>
#include <stdio.h>

#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>

extern UART_HandleTypeDef huart1;

#define SA818_TIMEOUT 1000

#define SA818_HL_PORT GPIOA
#define SA818_HL_PIN  GPIO_PIN_12

HAL_StatusTypeDef SA818_Init(void);
void SA818_SetHighPower(void);
void SA818_SetLowPower(void);

#endif /* SA818S_H_ */
