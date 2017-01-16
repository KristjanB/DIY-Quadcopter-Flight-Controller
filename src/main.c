
#include <ppm.h>
#include <stdint.h>
#include <stdbool.h>
#include "stdlib.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_timer.h"

#include "inc/hw_gpio.h"
#include "inc/hw_pwm.h"
#include "inc/hw_types.h"
#include "driverlib/pin_map.h"

#include "altitudehold.h"
#include "sonar.h"
#include "rx.h"
#include "time.h"
#include "buzzer.h"
#include "uart.h"
#include "i2c.h"
#include "mpu.h"
#include "mat.h"
#include "pid.h"
#include "gimbal.h"

#include "driverlib/timer.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/udma.h"
#include "driverlib/pwm.h"
#include "driverlib/ssi.h"
#include "driverlib/systick.h"
#include "utils/uartstdio.c"
#include <string.h>


#define MIN_YAW_SCALE -100.f // Max and Min rate of yaw
#define MAX_YAW_SCALE 100.f

// Program stuff...
#define CALIBRATE_ESC 	            0	// 1 -> To calibrate motors
#define REMOTE     		            1 	// 0 -> For debugging, so you don't need the remote

// Flight stuff...
#define FLIGHT_MODE_STAB 			0
#define FLIGHT_MODE_RATE            1
#define FLIGHT_MODE_ALTHOLD			0

#define COME_TO_PAPA				0   // 1 -> Shuts down motors when there is your hand under it.
#define KILL_SWITCH					1	  // 1 -> ON,                      not yet implemented
#define FRONT_OBSTACLE_AVOIDANCE	0

#if FLIGHT_MODE_STAB
float MAX_TX_SCALE = 35.f; // Max and Min angle of quadcopter
float MIN_TX_SCALE = -35.f;
#endif

#if FLIGHT_MODE_RATE
float MAX_TX_SCALE = 100.f; // Max and Min angle of quadcopter
float MIN_TX_SCALE = -100.f;
#endif

int main(){

 	SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
    initTime();
	initUart();
	initPWM();
	initI2C();
	initMPU6050();
	initRX();
	initBuzzer();
	initSonar();
	MPUtestConnection();
	IntMasterEnable();
	SysCtlDelay(3);
	float 	MAX = 1000.0f, MIN = -1000.0f;
	float	stabPitch = 0,
			PitchAngle,
			stabRoll = 0,
			RollAngle,
			Yaw = 0,
			Throttle;
	float 	motor1, motor2, motor3, motor4;
    float 	dtrate;
	double 	last = 0;
	float 	dt = 0.001;
	float 	rateRoll, ratePitch, rateYaw;
	float 	maxX=0, maxY=0;




#if CALIBRATE_ESC
	writeMotor1(MAX);
	writeMotor2(MAX);
	writeMotor3(MAX);
	writeMotor4(MAX);
	Delay(5000000);
	writeMotor1(MIN);
	writeMotor2(MIN);
	writeMotor3(MIN);
	writeMotor4(MIN);
	Delay(5000000);
#endif

#if REMOTE
	while(!isArmed()){

	}	// Wait until pilot arms quadcopter with remote
#endif



	while(1)
    {


     	RollAngle = getRXchannel(RX_ROLL);
     	PitchAngle = getRXchannel(RX_PITCH);
     	Yaw = getRXchannel(RX_YAW);
        Throttle = getRXchannel(RX_THROTTLE);
//
//        triggerGroundSonar();
//        float distance = getSonarActualDistance();
//        UARTprintf("%d  \n", (int)(distance));

#if FLIGHT_MODE_ALTHOLD

#endif


// This uses AUX2
#if KILL_SWITCH
        int kill = getRXchannel(RX_AUX2);
        if(kill > 0){
        	while(1){
				writeMotorsOFF();
				MixLed(0x02, 0x08, 0);
				BuzzerShort();
				Delay(500000);
				MixLed(0, 0, 0);
				BuzzerShort();
				Delay(300000);
		        int notkill = getRXchannel(RX_AUX2);
        		if(notkill < -900){
            		resetPID();
            		break;
        		}
        	}
        }
#endif


// This uses AUX1
#if COME_TO_PAPA
		triggerGroundSonar();
		float distance = getGroundSonar();
        uint32_t ValueTreshold = millis();
        int turnon = getRXchannel(RX_AUX1);
		if(distance > 0 && ValueTreshold > 1000 && turnon > 0){
			if(distance <= 15){
				BuzzerShort();
				while(1){
					writeMotorsOFF();
					MixLed(0x02, 0x08, 0x04);
					Delay(700000);
					MixLed(0, 0, 0);
					Delay(300000);
			        int turnoff = getRXchannel(RX_AUX1);
					if(turnoff < 0){
						resetPID();
						break;
					}
				}
			}
		}
#endif

#if FRONT_OBSTACLE_AVOIDANCE
		triggerGroundSonar();
		int turnon = 1; //getRXchannel(RX_AUX1);
		if(turnon > 0){
			float ObstacleDistance = getGroundSonar();
			if(ObstacleDistance <= 50){

			}
		   // UARTprintf("%d \n", (int)(ObstacleDistance));

		}

#endif

//    	float  maxAngle=100.0;
//    	maxY = getMPUangleY();
//    	maxX = getMPUangleX();
//    	if(maxX > maxAngle || maxY > maxAngle || maxX < -maxAngle || maxY < -maxAngle){
//    		writeMotorsOFF();
//    		while(1){
//    			//BuzzerShort();
//    			BlueLed(true);
//    			Delay(500000);
//    			BlueLed(false);
//    			Delay(100000);
//    		}
//    	}

    	double now = micros();
    	dtrate = (now - last) / 1e6f;
    	last = now;

        Yaw = mapf(Yaw, MIN, MAX, MIN_YAW_SCALE, MAX_YAW_SCALE);
        Yaw = constrain(Yaw, MAX_YAW_SCALE, MIN_YAW_SCALE);
	    RollAngle = mapf(RollAngle, MIN, MAX,MIN_TX_SCALE , MAX_TX_SCALE);
	    RollAngle = constrain(RollAngle, MAX_TX_SCALE, MIN_TX_SCALE);
	    PitchAngle = mapf(PitchAngle, MIN, MAX, MIN_TX_SCALE, MAX_TX_SCALE);
	 	PitchAngle = constrain(PitchAngle, MAX_TX_SCALE, MIN_TX_SCALE);

#if FLIGHT_MODE_STAB
	 	stabPitch = stabilizePID(PitchAngle, dtrate, 1);
	 	stabRoll = stabilizePID(RollAngle, dtrate, 2);
	 	stabPitch = constrain(stabPitch, MAX, MIN);
	 	stabRoll = constrain(stabRoll, MAX, MIN);
#endif

	 	// *****************   1 KHz loop time   ********************** //
     	if(dataReadyMPU()){

			readMPU();

#if FLIGHT_MODE_RATE
			stabPitch = PitchAngle;
			stabRoll = RollAngle;
#endif

			ratePitch = ratePID(stabPitch, dt, 1);
			rateRoll = ratePID(stabRoll, dt, 2);
			rateYaw = ratePID(Yaw, dt, 3);

			rateRoll = constrain(rateRoll, MAX, MIN);
			ratePitch = constrain(ratePitch, MAX, MIN);
			rateYaw = constrain(rateYaw, MAX, MIN);


			motor4 = Throttle - rateRoll - ratePitch - rateYaw;
			motor2 = Throttle + rateRoll - ratePitch + rateYaw;
			motor3 = Throttle - rateRoll + ratePitch + rateYaw;
			motor1 = Throttle + rateRoll + ratePitch - rateYaw;

			motor2 = constrain(motor2, MAX, MIN);
			motor1 = constrain(motor1, MAX, MIN);
			motor4 = constrain(motor4, MAX, MIN);
			motor3 = constrain(motor3, MAX, MIN);

			writeMotor1(motor1);
			writeMotor2(motor2);
			writeMotor4(motor4);
			writeMotor3(motor3);
	 }
  }
}
