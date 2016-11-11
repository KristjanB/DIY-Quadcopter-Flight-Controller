#include <stdint.h>
#include <stdbool.h>
#include "stdlib.h"
#include "mpu.h"
#include "uart.h"
#include "pid.h"
#include "mat.h"
#include "rx.h"


// rate pid PITCH/ROLL
float 	P = 3.53, // 3
	I = 0.0,
	D = 1.019; // 0.27

// stabilize pid ( heavy battery ) PITCH
float Kp= 0.36,		// gopro- 0.36 //without 0.441
	 Ki= 1.255, 	// gopro - 1.255 // without 1.771
	  Kd= 0.;

// rate pid YAW
float Prz = 3.,
	  Irz = 0.95,
	  Drz = 0.;

// stabilize pid YAW
float Psz = 0.,
	  Isz = 0.,
	  Dsz = 0.;

// stabilize pid (light battery)
//float Kp1 = 0.00,
//	  Ki1 = 0.00,
//	  Kd1 = 0.00;

float deltaErrSx = 0, LastErrorSx = 0,  errorSx, deltaErrSy = 0, LastErrorSy = 0,  errorSy;
float deltaErrRx = 0, LastErrorRx = 0,  errorRx, deltaErrRy = 0, LastErrorRy = 0,  errorRy ;
float deltaErrRz = 0, LastErrorRz = 0,  errorRz, deltaErrSz = 0, LastErrorSz = 0,  errorSz;
float MAX = 1000.0f, MIN = -1000.0f;
double errSumSx, errSumRx, errSumRz, errSumSy, errSumRy, errSumSz;
	float ratePID(float setpoint, float dt, int axle){
		// X axle
		if(axle == 1){
			float gyroX = getGyroX() * 57.295f;
			errorRx = setpoint - gyroX;
			errSumRx += errorRx * dt;
			errSumRx = constrain(errSumRx, MAX, MIN);
			deltaErrRx = (errorRx - LastErrorRx) / dt;
			LastErrorRx = errorRx;
			float Kr = P * errorRx;
			Kr = constrain(Kr, MAX,MIN);
			return (Kr + (I * errSumRx) + (D * deltaErrRx));
		}

		// Z axle
		if(axle == 3){
			float gyroZ = getGyroZ();
			errorRz = setpoint - gyroZ;
			errSumRz += errorRz * dt;
			errSumRz = constrain(errSumRz, MAX, MIN);
			deltaErrRz = (errorRz - LastErrorRz) / dt;
			LastErrorRz = errorRz;
			float Kr = Prz * errorRz;
			Kr = constrain(Kr, MAX,MIN);
			return (Kr + (Irz * errSumRz) + (Drz * deltaErrRz));
		}
		// Y axle
		else{
			float gyroY = getGyroY() * 57.295f;
			errorRy = setpoint - gyroY;
			errSumRy += errorRy * dt;
			errSumRy = constrain(errSumRy, MAX, MIN);
			deltaErrRy = (errorRy - LastErrorRy) / dt;
			LastErrorRy = errorRy;
			float Kr = P * errorRy;
			Kr = constrain(Kr, MAX,MIN);
			return (Kr + (I * errSumRy) + (D * deltaErrRy));
		}
	}

	float stabilizePID(float setpoint, float dt, int axle){

		float Pk = getRXchannel(RX_AUX1);
		Pk = mapf(Pk, -1000., 1000., 0., 2.);
		float Ik = getRXchannel(RX_AUX2);
		Ik = mapf(Ik, -1000, 1000, 0., 2.);
		// X axle
		if(axle == 1){
			float angleX = getMPUangleX() * 57.29577951f;
			errorSx = setpoint - angleX;	//
			errSumSx += errorSx * dt;
			errSumSx = constrain(errSumSx, MAX, MIN);
			deltaErrSx = (errorSx - LastErrorSx) / dt;
			LastErrorSx = errorSx;
			float Kpart = Kp * errorSx;
			Kpart = constrain(Kpart, MAX,MIN);
			return ((Kpart) + (Ki * errSumSx) + (Kd * deltaErrSx));
		}
		// Z axle
		if(axle == 3){
			float gyroZ = getGyroZ();
			errorSz = setpoint - gyroZ;
			errSumSz += errorSz * dt;
			errSumSz = constrain(errSumSz, MAX, MIN);
			deltaErrSz = (errorSz - LastErrorSz) / dt;
			LastErrorSz = errorSz;
			float Kr = Psz * errorSz;
			Kr = constrain(Kr, MAX,MIN);
			return (Kr + (Isz * errSumRz) + (Dsz * deltaErrRz));
		}
		// Y axle
		else{
			float angleY = getMPUangleY() * 57.29577951f;
			errorSy = setpoint - angleY;	//
			errSumSy += errorSy * dt;
			errSumSy = constrain(errSumSy, MAX, MIN);
			deltaErrSy = (errorSy - LastErrorSy) / dt;
			LastErrorSy = errorSy;
			float Kpart = Pk * errorSy;
			Kpart = constrain(Kpart, MAX,MIN);
			return ((Kpart) + (Ik * errSumSy) + (Kd* deltaErrSy));
		}

	}
