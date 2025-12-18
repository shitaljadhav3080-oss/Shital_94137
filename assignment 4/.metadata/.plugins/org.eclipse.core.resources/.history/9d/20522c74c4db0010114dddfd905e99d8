/*
 * my_led.c
 *
 *  Created on: Dec 17, 2025
 *      Author: kiran_z6dopa8
 */

#include "my_led.h"

void led_init(void)
{
	//0. enable clock for GPIOD in AHB1
	RCC->AHB1ENR |= BV(3);
	//1. select mode as output
	GPIOD->MODER &= ~(BV(25) | BV(27) | BV(29) | BV(31));
	GPIOD->MODER |= BV(24) | BV(26) | BV(28) | BV(30);
	//2. select type as push pull
	GPIOD->OTYPER &= ~(BV(12) | BV(13) | BV(14) | BV(15));
	//3. select speed as low
	GPIOD->OSPEEDR &= ~(BV(25) | BV(27) | BV(29) | BV(31));
	GPIOD->OSPEEDR &= ~(BV(24) | BV(26) | BV(28) | BV(30));
	//4. select pull up/down as no
	GPIOD->PUPDR &= ~(BV(25) | BV(27) | BV(29) | BV(31));
	GPIOD->PUPDR &= ~(BV(24) | BV(26) | BV(28) | BV(30));
}
void led_on(void)
{
	GPIOD->ODR |= BV(12)| BV(13) | BV(14) | BV(15);
}
void led_off(void)
{
	GPIOD->ODR &= ~(BV(12)| BV(13) | BV(14) | BV(15));
}

