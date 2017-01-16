/*
 * rx.c
 *
 *  Created on: 1 Mar 2016
 *      Author: Kristjan
 */
#include <ppm.h>
#include <stdint.h>
#include <stdbool.h>


#include "buzzer.h"
#include "rx.h"
#include "time.h"
#include "mat.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"


static volatile uint16_t rxChannel[RX_NUM_CHANNELS];

unsigned long RisingEdge1 = 0;
unsigned long RisingEdge2 = 0;
unsigned long RisingEdge3 = 0;
unsigned long RisingEdge4 = 0;
unsigned long RisingEdge5 = 0;
unsigned long RisingEdge6 = 0;



 void RXChannel(){


	 uint8_t IntStatus = GPIOIntStatus(GPIO_PORTA_BASE, true); // GET INTERRUPT STATUS ( ON WHICH PINS INTERRUPT HAPPENED)
	GPIOIntClear(GPIO_PORTA_BASE, IntStatus);         // CLEAR INT FLAG


	///////////        RX_ROLL       ///////////////
	if((IntStatus & GPIO_INT_PIN_2) == GPIO_INT_PIN_2)// DID INTERRUPT OCCURED ON PIN A3?
	{

		if(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_2)  == GPIO_PIN_2)
		{
			RisingEdge1 = micros();
		}
		else if(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_2) == 0)
		{
			uint32_t i = micros();
			rxChannel[0] = i - RisingEdge1;

		}
	}


		/////////// RX_PITCH_ /////////////
	if((IntStatus & GPIO_INT_PIN_3) == GPIO_INT_PIN_3)
	{
		if(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_3) == GPIO_PIN_3)
		{
			RisingEdge2 = micros();
		}
		else if(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_3) == 0)
		{
			uint32_t i = micros();
			rxChannel[1] = (i - RisingEdge2);
		}
	}


	/////////// THROTTLE /////////////

	if((IntStatus & GPIO_INT_PIN_4) == GPIO_INT_PIN_4)
	{
		if(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_4) == GPIO_PIN_4)
		{
			RisingEdge3 = micros();
		}
		else if(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_4) == 0)
		{
			uint32_t i = micros();
			rxChannel[2] = (i - RisingEdge3);
		}
	}

	/////////// AUX YAW /////////////
	if((IntStatus & GPIO_INT_PIN_5) == GPIO_INT_PIN_5)
	{
		if(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_5) == GPIO_PIN_5)
		{
			RisingEdge4 = micros();
		}
		else if(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_5) == 0)
		{
			uint32_t i = micros();
			rxChannel[3] = (i - RisingEdge4);
		}
	}

	/////////// AUX5 /////////////
	if((IntStatus & GPIO_INT_PIN_6) == GPIO_INT_PIN_6)
	{
		if(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6)
		{
			RisingEdge5 = micros();
		}
		else if(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == 0)
		{
			uint32_t i = micros();
			rxChannel[4] = (i - RisingEdge5);
		}
	}

	/////////// AUX6 /////////////
	if((IntStatus & GPIO_INT_PIN_7) == GPIO_INT_PIN_7)
	{
		if(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_7) == GPIO_PIN_7)
		{
			RisingEdge6 = micros();
		}
		else if(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_7) == 0)
		{
			uint32_t i = micros();
			rxChannel[5] = (i - RisingEdge6);
		}
	}



}




void initRX(){

	  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	  SysCtlDelay(3);


	  GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
	  GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_BOTH_EDGES);
	  GPIOIntRegister(GPIO_PORTA_BASE, RXChannel);
	  IntPrioritySet(INT_GPIOA, 0);
	  GPIOIntEnable(GPIO_PORTA_BASE, GPIO_INT_PIN_2 | GPIO_INT_PIN_3 | GPIO_INT_PIN_4 | GPIO_INT_PIN_5 | GPIO_INT_PIN_6 | GPIO_INT_PIN_7);
}

float getRXchannel(rxChannel_e channel){
	float value = rxChannel[channel] ;
	value = mapf(value, 1010, 2000, -1000.f, 1000.f);
	value = constrain(value, 1000.f, -1000.f);
	return value;
//	float filteredValue = LowPassFilter(value);
//	filteredValue = mapf(filteredValue, 1010, 2000, -1000.f, 1000.f);
//	filteredValue = constrain(filteredValue, 1000.f, -1000.f);
//	return filteredValue;		// CHECK MAX AND MIN RANGE OF TRANSMITTER!!!!!!!! mapf(rxChannel[channel], 1034, 2022, -100.0f, 100.0f);
}




// ARM routine. Write Lowest value to motors to arm them.
int isArmed(){
	float value;
	float T = getRXchannel(RX_THROTTLE);
	float Y = getRXchannel(RX_YAW);
	if((T < -950.) && (Y > 950.)){
		value = 1;
		Delay(500000);
        writeMotorsOFF(); // Arm and off is the same. OR IS IT?! I don't know.
		RedLed(true);
		Delay(500000);
		RedLed(false);
		return value;
	}
	else{
		value = 0;
		return value;
	}
}

