#include <stdint.h>
#include <stdbool.h>
#include "stdlib.h"
#include "mpu.h"
#include "uart.h"
#include "pid.h"
#include "mat.h"
#include "rx.h"

#define PID_TUNING		1

// rate pid PITCH/ROLL
float 	P = 2.01;

// stabilize pid ( heavy battery ) PITCH
float Kp= 8,		// gopro- 0.36 //without 3.18
	 Ki= 0.01; 	// gopro - 1.255 // without 0.065

// rate pid YAW
float Prz = 12.32,
	  Irz = 17.25;


float LastErrorSx = 0,  errorSx, LastErrorSy = 0,  errorSy;
float errorRx, errorRy;
float LastErrorRz = 0,  errorRz;
float MAX = 1000.0f, MIN = -1000.0f;
double errSumSx, errSumRz, errSumSy;

float Pk=0, Ik = 0;



float ratePID(float setpoint, float dt, int axle){
// X axle
		if(axle == 1){
			float gyroX = getGyroX();
			errorRx = setpoint - gyroX;
			float Kr = P * errorRx;
			Kr = constrain(Kr, MAX,MIN);
			return Kr;
		}

		// Z axle
		 if(axle == 3){
		 	float gyroZ = getGyroZ();
		 	errorRz = setpoint - gyroZ;
		 	errSumRz += errorRz * dt;
		 	errSumRz = constrain(errSumRz, MAX, MIN);
		 	float Kr = Prz * errorRz;
		 	Kr = constrain(Kr, MAX,MIN);
		 	return (Kr + (Irz* errSumRz));
		 }
		// Y axle
		else{
			float gyroY = getGyroY();
			errorRy = setpoint - gyroY;
			float Kr = P * errorRy;
			Kr = constrain(Kr, MAX,MIN);
			return Kr;
		}
	}

float stabilizePID(float setpoint, float dt, int axle){
#if PID_TUNING
	Pk = getRXchannel(RX_AUX1);
	Pk = mapf(Pk, -1000., 1000., 0., 15.);
	Ik = getRXchannel(RX_AUX2);
	Ik = mapf(Ik, -1000, 1000, 0., 0.5);
#endif

		// X axle
		if(axle == 1){
			float angleX = getMPUangleX();
			errorSx = setpoint - angleX;	//
			errSumSx += errorSx * dt;
			errSumSx = constrain(errSumSx, MAX, MIN);
			float Kpart = Kp* errorSx;
			Kpart = constrain(Kpart, MAX,MIN);
			return ((Kpart) + (Ki* errSumSx));
		}

		else{
			float angleY = getMPUangleY();
			errorSy = setpoint - angleY;	//
			errSumSy += errorSy * dt;
			errSumSy = constrain(errSumSy, MAX, MIN);
			float Kpart = Kp * errorSy;
			Kpart = constrain(Kpart, MAX,MIN);
			return ((Kpart) + (Ki * errSumSy));
		}

}

float SonarError;
float Kp_sonar = 0.00;

float altitudeholdSonarPID(float setpoint, float dt){
	float distance = getSonarActualDistance();
	SonarError = setpoint - distance;
	return (Kp_sonar * SonarError);
}


float AccError, AccErrorSum, AccErrorDelta, AccErrorLast;
float Kp_acc = 0.00,
	  Ki_acc = 0.00,
	  Kd_acc = 0.00;

float altitudeholdAccPID(float setpoint, float dt){
	float acc = getAccZ();
	AccError = setpoint - acc;
	AccErrorSum += AccError * dt;
	AccErrorSum  = constrain(AccErrorSum, MAX, MIN);
	AccErrorDelta = (AccError - AccErrorLast) / dt;
	AccErrorLast = AccError;
	return (Kp_acc * AccError) +
		   (Ki_acc * AccErrorSum) +
		   (Kd_acc * AccErrorDelta);
}


void resetPID(){
	 errorSy = 0, errorSx = 0, errorRx = 0, errorRy = 0, errorRz = 0;
	 errSumSx = 0, errSumRz = 0, errSumSy = 0;
	 AccError = 0, AccErrorDelta = 0, AccErrorSum = 0, AccErrorLast = 0;
}





