/*
 * sonar.c
 *
 *  Created on: 1 Nov 2016
 *      Author: Kristjan
 */
#include <stdint.h>
#include <stdbool.h>

#include "buzzer.h"
#include "time.h"
#include "sonar.h"


#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"

double RisingEdgeG, RisingEdgeO;
float distanceGround, distanceObstacle;

float getGroundSonar(){
	float value = distanceGround / 58.f;
	return value;
}

float getObstacleSonar(){
	float value = distanceObstacle / 58.f;
	return value;
}

void triggerGroundSonar(void) {
    static uint32_t lastTrigger = 0;

    uint32_t now = millis();
    int diff = now - lastTrigger;
    if (diff >= 26) { // Trigger every 25ms
        lastTrigger = now;
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_PIN_5); // Set pin high
        Delay(10); // Other sources wait 10us
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0); // Set pin low

    }
}

void triggerObstacleSonar(void) {
    static uint32_t lastTrigger = 0;

    uint32_t now = millis();
    int diff = now - lastTrigger;
    if (diff >= 26) { // Trigger every 25ms
        lastTrigger = now;
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_PIN_5); // Set pin high
        Delay(10); // Other sources wait 10us
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0); // Set pin low

    }
}

void SonarInt(){
	uint8_t IntStatus = GPIOIntStatus(GPIO_PORTC_BASE, true); // GET INTERRUPT STATUS ( ON WHICH PINS INTERRUPT HAPPENED)
	GPIOIntClear(GPIO_PORTC_BASE, IntStatus);         // CLEAR INT FLAG

	// Ground Sonar
	if((IntStatus & GPIO_INT_PIN_6) == GPIO_INT_PIN_6)// DID INTERRUPT OCCURED ON PIN A3?
	{

		if(GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_6)  == GPIO_PIN_6)
		{
			RisingEdgeG = micros();
		}
		else if(GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_6) == 0)
		{
			uint32_t i = micros();
			distanceGround = i - RisingEdgeG;

		}
	}
	// Obstacle Sonar
	if((IntStatus & GPIO_INT_PIN_4) == GPIO_INT_PIN_4)// DID INTERRUPT OCCURED ON PIN A3?
	{

		if(GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4)  == GPIO_PIN_4)
		{
			RisingEdgeO = micros();
		}
		else if(GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4) == 0)
		{
			uint32_t i = micros();
			distanceObstacle = i - RisingEdgeO;

		}
	}
}



void initSonar(){
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

	GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_6 | GPIO_PIN_4); // echo
	GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_7); // trigger
	GPIOIntTypeSet(GPIO_PORTC_BASE, GPIO_PIN_6 | GPIO_PIN_4, GPIO_BOTH_EDGES);
	GPIOIntRegister(GPIO_PORTC_BASE, SonarInt);
	IntPrioritySet(INT_GPIOC, 0);
	GPIOIntEnable(GPIO_PORTC_BASE, GPIO_INT_PIN_6 | GPIO_INT_PIN_4);
	SysCtlDelay(3);
}

