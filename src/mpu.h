/*
 * mpu.h
 *
 *  Created on: 18 Apr 2016
 *      Author: Kristjan
 */

#ifndef QUADCOPTER_MPU6050_H_
#define QUADCOPTER_MPU6050_H_

void MPUtestConnection();
void initMPU6050();
void readMPU();
float getAccZ();
float getMPUangleY();
float getMPUangleX();
float getGyroX();
float getGyroY();
float getGyroZ();
int dataReadyMPU();
void calibrateMPU();

#endif /* QUADCOPTER_MPU6050_H_ */
