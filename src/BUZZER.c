/*
 * BUZZER.c
 *
 *  Created on: 11 Mar 2016
 *      Author: Kristjan
 */


#include <stdint.h>
#include <stdbool.h>
#include "BUZZER.h"
#include "TIME.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"


void initBuzzer(){
	  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	  SysCtlDelay(3);
	  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_1);
//	  GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1);

}


void GreenLed(bool enable)
{
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, enable ? GPIO_PIN_3 : 0);
}

void RedLed(bool enable)
{
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, enable ? GPIO_PIN_1 : 0);
}

void BlueLed(bool enable)
{
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, enable ? GPIO_PIN_2: 0);
}

void MixLed(int R, int G, int B){
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, R);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, B);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, G);
}



void BuzzerShort(){
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1);
	Delay(500000);
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0);
	Delay(500000);
}

void BuzzerLong(){
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1);
	Delay(1500000);
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0);
	Delay(1500000);
}
