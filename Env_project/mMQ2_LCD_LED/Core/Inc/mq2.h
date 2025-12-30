/*
 * mq2.h
 *
 *  Created on: Dec 30, 2025
 *      Author: Admin
 */

#ifndef INC_MQ2_H_
#define INC_MQ2_H_
#include "main.h"
/* Standard C libraries */
#include <stdio.h>      // for
#include <string.h>     // for string handling
#include <stdarg.h>     // for variable arguments

/* STM32 HAL library */
#include "stm32f4xx_hal.h"
void MX_ADC1_Init(void);
uint16_t read_adc(void);
extern ADC_HandleTypeDef hadc1;
#endif /* INC_MQ2_H_ */
