/*
 * slave_side.h
 *
 *  Created on: Feb 4, 2026
 *      Author: Amal
 */

#ifndef INC_SLAVE_SIDE_H_
#define INC_SLAVE_SIDE_H_

#include "stm32f4xx_hal.h"

/* ================= LORA GPIO PINS ================= */
#define LORA_NSS_PORT GPIOA
#define LORA_NSS_PIN  GPIO_PIN_15

#define LORA_BUSY_PORT GPIOD
#define LORA_BUSY_PIN  GPIO_PIN_5

#define LORA_RST_PORT GPIOD
#define LORA_RST_PIN  GPIO_PIN_2

#define ROBOT_UID 5678

/* ================= DEBUG LED PINS ================= */
#define DBG_PORT         GPIOE
#define DBG_WAKE_PIN     GPIO_PIN_2
#define DBG_SLEEP_PIN    GPIO_PIN_3
#define DBG_CONT_PIN     GPIO_PIN_4
#define DBG_DISC_PIN     GPIO_PIN_0  // Reusing NSS LED for discrete
#define DBG_OFF_PIN      GPIO_PIN_1  // Reusing RST LED for off

/* ================= ROBOT IDENTIFICATION ================= */
#define ROBOT_UID        5678  // Change this for each robot
#define UID_BROADCAST    0xFFFF

/* ================= FUNCTION PROTOTYPES ================= */
void Slave_App_Init(void);
void Slave_App_Loop(void);
void Motor_Task(void);
void LORA_RX_Poll(void);

void lora_write_reg(uint16_t reg, uint8_t value);

#endif /* INC_SLAVE_SIDE_H_ */
