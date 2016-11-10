#include <stdint.h>
#include <stdbool.h>

#include "time.h"
#include "pwm.h"
#include "mat.h"

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/pwm.h"


int period;

void initPWM()
{
	SysCtlPWMClockSet(SYSCTL_PWMDIV_32); // Set divider to 32

    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0); // Enable PWM peripheral
    SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); // Enable GPIOB peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

    SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated

    // Use alternate function

    GPIOPinConfigure(GPIO_PB7_M0PWM1);
    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinConfigure(GPIO_PB5_M0PWM3);
    GPIOPinConfigure(GPIO_PB4_M0PWM2);
    GPIOPinConfigure(GPIO_PC5_M0PWM7);

    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7 | GPIO_PIN_6 | GPIO_PIN_5 | GPIO_PIN_4); // Use pin with PWM peripheral
    GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_5);
    // Configure the PWM generator for count down mode with immediate updates to the parameters
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

    // The period is set to 25ms
    period = 50000;

    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, period); // Set the period
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, period);

    // Start the timers in generator 0 and 1
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);

//
//    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, 800 );
    // Enable the outputs
    PWMOutputState(PWM0_BASE,PWM_OUT_1_BIT | PWM_OUT_0_BIT |  PWM_OUT_2_BIT |  PWM_OUT_3_BIT | PWM_OUT_7_BIT, true);
}

// LEFT
void writeMotorL(float value){
	value = mapf(value, -1000., 1000., 2660, 4660);
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, value);
}
// RIGHT
void writeMotorR(float value){
	value = mapf(value, -1000., 1000., 2660, 4660);
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, value);
}
// BACK
void writeMotorB(float value){
	value = mapf(value, -1000., 1000., 2660, 4660);
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, value);
}
// FRONT
void writeMotorF(float value){
	value = mapf(value, -1000., 1000., 2660, 4660);
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, value);
}
