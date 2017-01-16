#include <ppm.h>
#include <stdint.h>
#include <stdbool.h>

#include "time.h"
#include "mat.h"
#include "rx.h"

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/pwm.h"

#define MIN_PPM 20200.f // This value is "how many clock ticks does 1030us takes" div4 - 20600(1010us)
#define MAX_PPM 37200.f // Same here but 1860 us - From Afro ESC datasheet. 	  div4 - 37200(1860us)

double period = 50000; // Period of PWM is set to 2.5ms/400Hz with divider 4


void initPWM()
{
	SysCtlPWMClockSet(SYSCTL_PWMDIV_4);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0); // Enable PWM peripheral

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); // Enable GPIOB peripheral
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC); // Enable GPIOC peripheral for sonar

		SysCtlDelay(3); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated

    GPIOPinConfigure(GPIO_PB7_M0PWM1);
    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinConfigure(GPIO_PB5_M0PWM3);
    GPIOPinConfigure(GPIO_PB4_M0PWM2);
//    GPIOPinConfigure(GPIO_PC5_M0PWM7); // Servo motor - gimbal.

    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7 | GPIO_PIN_6 | GPIO_PIN_5 | GPIO_PIN_4); // Use pin with PWM peripheral
//    GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_5);

    // Configure the PWM generator for count down mode with immediate updates to the parameters
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
//    PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

  	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, period);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, period);
//    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, 32258);


    // Start the timers in generator 0 and 1
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);
//    PWMGenEnable(PWM0_BASE, PWM_GEN_2);

//    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, 3000);

    PWMOutputState(PWM0_BASE,PWM_OUT_1_BIT | PWM_OUT_0_BIT |  PWM_OUT_2_BIT |  PWM_OUT_3_BIT, true);

//	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7_BIT, 13);
}

// LEFT
void writeMotor1(float value){
	value = mapf(value, -1000., 1000., MIN_PPM, MAX_PPM);
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, value);
}
// RIGHT
void writeMotor2(float value){
	value = mapf(value, -1000., 1000., MIN_PPM, MAX_PPM);
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, value);
}
// BACK
void writeMotor3(float value){
	value = mapf(value, -1000., 1000., MIN_PPM, MAX_PPM);
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, value);
}
// FRONT
void writeMotor4(float value){
	value = mapf(value, -1000., 1000., MIN_PPM, MAX_PPM);
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, value);
}
void writeMotorsOFF(){
	writeMotor4(-1000);
	writeMotor3(-1000);
	writeMotor2(-1000);
	writeMotor1(-1000);
}

void writeGimbal(float value){
	value = mapf(value, -1000., 1000., 2500., 5000.); // out is 1ms-2ms
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0_BIT, value);
}

void writeMotorsSlowlyOFF(float throttle){

	int a = throttle;
	do{
		writeMotor4(a);
		writeMotor3(a);
		writeMotor2(a);
		writeMotor1(a);
		a -= 1;
	}while(a > -1000);
}
