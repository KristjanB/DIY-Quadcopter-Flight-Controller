/*
 * mat.c
 *
 *  Created on: 14 May 2016
 *      Author: Kristjan
 */

#include <math.h>
#include "mat.h"

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
    float value = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; // From Arduino source code: https://github.com/arduino/Arduino/blob/ide-1.5.x/hardware/arduino/avr/cores/arduino/WMath.cpp
    return value;
}

float constrain(float value, float max, float min)
{
	if(value>max) value = max;
	if(value<min) value = min;
	return value;
}
