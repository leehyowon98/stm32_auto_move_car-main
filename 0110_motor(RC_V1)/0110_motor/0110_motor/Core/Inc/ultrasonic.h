/*
 * ultrasonic.h
 *
 *  Created on: Jan 20, 2025
 *      Author: user
 */

#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "delay.h"

// 트리거 핀 및 에코 핀 설정
#define TRIG_PORT1 GPIOA
#define TRIG_PIN1  GPIO_PIN_5
#define ECHO_PORT1 GPIOA
#define ECHO_PIN1  GPIO_PIN_0

#define TRIG_PORT2 GPIOB
#define TRIG_PIN2  GPIO_PIN_6
#define ECHO_PORT2 GPIOB
#define ECHO_PIN2  GPIO_PIN_8

#define TRIG_PORT3 GPIOB
#define TRIG_PIN3  GPIO_PIN_7
#define ECHO_PORT3 GPIOB
#define ECHO_PIN3  GPIO_PIN_9

void HCSR04_Init(void);
void HCSR04_Read(void);
uint16_t Get_Distance1(void);
uint16_t Get_Distance2(void);
uint16_t Get_Distance3(void);

#endif /* ULTRASONIC_H */
