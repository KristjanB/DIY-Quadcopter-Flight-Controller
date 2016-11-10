/*
 * time.h
 *
 *  Created on: 8 Mar 2016
 *      Author: Kristjan
 */

#ifndef QUADCOPTER_TIME_H_
#define QUADCOPTER_TIME_H_


void Delay(uint32_t delay);

void initTime();

uint32_t millis();

uint32_t micros(void);

#endif /* QUADCOPTER_TIME_H_ */
