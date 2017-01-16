/*
 * pwm.h
 *
 *  Created on: 13 Mar 2016
 *      Author: Kristjan
 */

#ifndef PPM_H_
#define PPM_H_

void initPWM();

void writeMotor1(float value);
void writeMotor2(float value);
void writeMotor3(float value);
void writeMotor4(float value);
void writeMotorsOFF();
void writeGimbal(float value);
void writeMotorsSlowlyOFF(float throttle);

#endif /* QUADCOPTER_PWM_H_ */
