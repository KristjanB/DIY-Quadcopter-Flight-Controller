/*
 * SONAR.c
 *
 *  Created on: 1 Nov 2016
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



void SonarInt(){

}



void initSonar(){
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	SysCtlDelay(3);
	GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_6); // echo
	GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_5); // trigger
	GPIOIntRegister(INT_GPIOC, SonarInt);
	GPIOIntEnable(GPIO_PORTC_BASE, GPIO_INT_PIN_5);
}

