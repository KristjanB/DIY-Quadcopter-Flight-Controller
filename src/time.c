/*
 * time.c
 *
 *  Created on: 8 Mar 2016
 *      Author: Kristjan
 */
#include <stdint.h>
#include <stdbool.h>

#include "time.h"

#include "driverlib/systick.h"

volatile long microsec;

static void SysTickIntHandler(void){
	microsec++;
}

void Delay(uint32_t delay){
	volatile long i = microsec;
	while((microsec-i) < delay);
}

void initTime()
{
	  SysTickPeriodSet(79);
	  SysTickIntRegister(SysTickIntHandler);
	  SysTickIntEnable();
	  SysTickEnable();
}

uint32_t millis() {
    return microsec / 1000UL;
}

uint32_t micros(void) {
	return microsec;
}
