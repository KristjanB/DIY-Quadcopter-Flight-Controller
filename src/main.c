
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

#include "rx.h"
#include "time.h"
#include "buzzer.h"
#include "uart.h"
#include "i2c.h"
#include "mpu.h"
#include "pwm.h"
#include "mat.h"
#include "pid.h"
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

int main()
{

 	SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);

 	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
 	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_4);
    initTime();
	initUart();
	initI2C();
	initMPU6050();
	initPWM();
	initRX();
	initBuzzer();
	MPUtestConnection();
	IntMasterEnable();
	SysCtlDelay(3);

	float MAX = 1000.0f, MIN = -1000.0f;
	float  	stabilizePitch = 0,
			PitchOut = 0,
			Pitch,
			stabilizeRoll = 0,
			RollOut = 0,
			Roll = 0,
			Yaw = 0,
			YawOut = 0,
			stabilizeYaw = 0,
			Throttle;
	int		R = 0x2,
	 	 	G = 0x8,
	 	 	B = 0x4;
	float 	motorF,
			motorB,
			motorR,
			motorL;
    float 	dtrate;
	double 	last = 0;
	float 	dt = 0.001;
	int blueled = true;
	int greenled = false;
	int k = 1;

//*******ESC CALIBRATION ROUTINE *******//
////
//	writeMotorL(MAX);
//	writeMotorR(MAX);
//	writeMotorB(MAX);
//	writeMotorF(MAX);
//	Delay(5000000);
//	writeMotorR(MIN);
//	writeMotorL(MIN);
//	writeMotorB(MIN);
//	writeMotorF(MIN);
//	Delay(5000000);

	while(!isArmed())	// Wait untill pilot arms quadcopter with remote
	{

	}

    while(1)
    {
    	double now = micros();
    	dtrate = (now - last) / 1e6f;
    	last = now;

    	Roll = getRXchannel(RX_ROLL);
    	Pitch = getRXchannel(RX_PITCH);
    	Yaw = getRXchannel(RX_YAW);

//    	UARTprintf("%d %d %d\n" ,(int)(Yaw), (int)(Roll), (int)(Pitch));
    	Yaw = mapf(Yaw, MIN, MAX, 250.f, -250.f);
		Roll = mapf(Roll, MIN, MAX, 80.f, -80.f);
		Pitch = mapf(Pitch, MIN, MAX, -80.f, 80.f);

		Yaw = constrain(Yaw, 250., -250.);
		Roll = constrain(Roll, 80., -80.);
		Pitch = constrain(Pitch, 80., -80.);

		YawOut = ratePID(Yaw, dtrate, 3);
		PitchOut = ratePID(Pitch, dtrate, 1);
		RollOut = ratePID(Roll, dtrate, 2);

		RollOut = constrain(RollOut, MAX, MIN);
		YawOut = constrain(YawOut, MAX, MIN);
		PitchOut = constrain(PitchOut, MAX, MIN);

    	if(dataReadyMPU()){

			readMPU();

			Throttle = getRXchannel(RX_THROTTLE);

    		stabilizePitch =stabilizePID(PitchOut, dt, 1);
    		stabilizeRoll = stabilizePID(RollOut, dt, 2);
    		stabilizeYaw = stabilizePID(YawOut, dt, 3);

    		stabilizeRoll = constrain(stabilizeRoll, MAX, MIN);
    		stabilizePitch = constrain(stabilizePitch, MAX, MIN);
    		stabilizeYaw = constrain(stabilizeYaw, MAX, MIN);

//    		    	UARTprintf("%d %d %d\n" ,(int)(Yaw), (int)(Roll), (int)(Pitch));
//    		UARTprintf("%d \n", (int)(YawOut));

			motorF = Throttle - stabilizeRoll - stabilizePitch + YawOut;
    		motorR =  Throttle + stabilizeRoll - stabilizePitch - YawOut;
    		motorB = Throttle + stabilizeRoll + stabilizePitch + YawOut;
    		motorL = Throttle - stabilizeRoll +stabilizePitch - YawOut;

    		motorR = constrain(motorR, 1000.0f, -1000.0f);
    		motorL = constrain(motorL, 1000.0f, -1000.0f);
    		motorF = constrain(motorF, 1000.0f, -1000.0f);
    		motorB = constrain(motorB, 1000.0f, -1000.0f);

			writeMotorL(motorL);
	 		writeMotorR(motorR);
	 		writeMotorF(motorF);
	 		writeMotorB(motorB);
		}
    }
}
