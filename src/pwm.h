/*
 * pwm.h
 *
 *  Created on: 13 Mar 2016
 *      Author: Kristjan
 */

#ifndef QUADCOPTER_PWM_H_
#define QUADCOPTER_PWM_H_

void initPWM();

void writeMotorL(float value);
void writeMotorR(float value);
void writeMotorB(float value);
void writeMotorF(float value);

#endif /* QUADCOPTER_PWM_H_ */
