/*
 * pid.h
 *
 *  Created on: 12 Oct 2016
 *      Author: Kristjan
 */

#ifndef PID_H_
#define PID_H_

float ratePID(float setpoint, float dt, int axle);
float stabilizePID(float setpoint, float dt, int axle);
void resetPID();
float altitudeholdSonarPID(float setpoint, float dt);


#endif /* PID_H_ */
