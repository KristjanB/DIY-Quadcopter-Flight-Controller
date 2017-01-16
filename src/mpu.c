#include <stdint.h>
#include <stdbool.h>

#include "i2c.h"
#include "time.h"
#include "buzzer.h"
#include "mpu.h"
#include "math.h"
#include "uart.h"

#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"


#define MPU_ADDRESS		0x68 // AD0 pin na 3.3 V in pol je 0x69, cene nevem (oz niti ne dela (?????))

#define WHO_AM_I		0x75
#define PWR_MGMT_1		0x6B
#define SMPRT_DIV		0x19
#define CONFIG			0x1A
#define GYRO_CONFIG		0x1B
#define ACC_CONFIG		0x1C
#define INT_PIN_CFG		0x37
#define INT_ENABLE		0x38

#define ACCEL_XOUT_H	0x3B
#define ACCEL_XOUT_L	0x3C
#define ACCEL_YOUT_H	0x3D
#define ACCEL_YOUT_L	0x3E
#define ACCEL_ZOUT_H	0x3F
#define ACCEL_ZOUT_L	0x40

#define GYRO_XOUT_H		0x43
#define GYRO_XOUT_L		0x44
#define GYRO_YOUT_H		0x45
#define GYRO_YOUT_L		0x46
#define GYRO_ZOUT_H		0x47
#define GYRO_ZOUT_L		0x48

int WhoAmI, RegReset = 0;

int accXout_L, accXout_H, accXout;
int accYout_L, accYout_H, accYout;
int accZout_L, accZout_H, accZout;
float accXos[3] = {0,0,0};
float accYos[3]= {0,0,0};

int gyroXout_L, gyroXout_H, gyroXout;
int gyroYout_L, gyroYout_H, gyroYout;
int gyroZout_L, gyroZout_H, gyroZout;
float gyroXos[3] = {0,0,0};
float gyroYos[3] = {0,0,0};
float gyroZos;
float fiXos[3]={0,0,0};
float fiYos[3]= {0,0,0};
float kotXos;

float 	accXoffset = 7.,
		accYoffset = -2.,
		accZoffset = 16387.,
		gyroXoffset = -1.,
		gyroYoffset = 0.,
		gyroZoffset = 0.;

void MPUtestConnection()
{
	readI2C(MPU_ADDRESS, WHO_AM_I, &WhoAmI);
	if(WhoAmI == 0x68)
	{
//		UARTprintf("Connection succesful ! \n");
		GreenLed(true);
//		BuzzerShort(true);
		Delay(1000000);
		GreenLed(false);
	}
	else {
		while(1){
			GreenLed(true);
			Delay(500000);
			GreenLed(false);
			Delay(400000);

		}
	}
}

#define tip 0.001
#define tau 1.6 // 1.6 je od turka.
#define ett 0.999375195
int data_ready;
float Itemp;


void readMPU()
{
//	uint32_t status=0;
//	status = GPIOIntStatus(GPIO_PORTE_BASE,true);
//	GPIOIntClear(GPIO_PORTE_BASE, status);


	readI2C(MPU_ADDRESS, ACCEL_XOUT_H, &accXout_H);
	readI2C(MPU_ADDRESS, ACCEL_XOUT_L, &accXout_L);
	readI2C(MPU_ADDRESS, ACCEL_YOUT_H, &accYout_H);
	readI2C(MPU_ADDRESS, ACCEL_YOUT_L, &accYout_L);
	readI2C(MPU_ADDRESS, ACCEL_ZOUT_H, &accZout_H);
	readI2C(MPU_ADDRESS, ACCEL_ZOUT_L, &accZout_L);

	readI2C(MPU_ADDRESS, GYRO_XOUT_H, &gyroXout_H);
	readI2C(MPU_ADDRESS, GYRO_XOUT_L, &gyroXout_L);
	readI2C(MPU_ADDRESS, GYRO_YOUT_H, &gyroYout_H);
	readI2C(MPU_ADDRESS, GYRO_YOUT_L, &gyroYout_L);
	readI2C(MPU_ADDRESS, GYRO_ZOUT_H, &gyroZout_H);
	readI2C(MPU_ADDRESS, GYRO_ZOUT_L, &gyroZout_L);

	accXout = ((accXout_H << 8) | accXout_L);
	accYout = ((accYout_H << 8) | accYout_L);
	accZout = ((accZout_H << 8) | accZout_L);

	gyroXout = ((gyroXout_H << 8) | gyroXout_L);
	gyroYout = ((gyroYout_H << 8) | gyroYout_L);
	gyroZout = ((gyroZout_H << 8) | gyroZout_L);

	if(accXout&0x8000) accXout|=0xFFFF0000;
	if(accYout&0x8000) accYout|=0xFFFF0000;
	if(accZout&0x8000) accZout|=0xFFFF0000;

	if(gyroXout&0x8000) gyroXout|=0xFFFF0000;
	if(gyroYout&0x8000) gyroYout|=0xFFFF0000;
	if(gyroZout&0x8000) gyroZout|=0xFFFF0000;


	accXos[1]=accXos[0];
	accYos[1]=accYos[0];
	gyroXos[1]=gyroXos[0];
	gyroYos[1]=gyroYos[0];
	fiXos[2]=fiXos[1];
	fiXos[1]=fiXos[0];
	fiYos[2]=fiYos[1];
	fiYos[1]=fiYos[0];

	float a = 2*ett,
		  b = -ett*ett,
		  c = tip*ett/tau-ett+1,
		  d = ett*ett-tip*ett/tau-ett,
		  e = tip*ett;

	accXos[0] = -atan2(accXout, accZout) ;
	accYos[0] = -atan2(accYout, accZout) ;

	gyroXos[0] = (float)gyroYout * 0.00106422515365507901031932363932f;		// pi/(180*16.4) -> Trans. into rad + sensitivity
	gyroYos[0] = -(float)gyroXout * 0.00106422515365507901031932363932f;
	gyroZos = (float)gyroZout/16.4; // Already in deg/s, we don't need rad.

	fiXos[0] = a*fiXos[1] + b*fiXos[2] + c*accXos[0] + d*accXos[1] + e*(gyroXos[0]-gyroXos[1]);
	fiYos[0] = a*fiYos[1] + b*fiYos[2] + c*accYos[0] + d*accYos[1] + e*(gyroYos[0]-gyroYos[1]);
	UARTprintf("%d\n", (int)(fiXos[0]*57)); //accZout/4096

//	UARTprintf("%d %d \n", (int)(fiXos[0]*57), (int)(fiYos[0]*57));
	//***********************************************************************

}


#define RESET_MPU 0

void initMPU6050()
{
	#if RESET_MPU
	writeI2C(MPU_ADDRESS, PWR_MGMT_1, (1 << 7)); // Reset device, this resets all internal registers to their default values
    Delay(100);
    do{
    	readI2C(MPU_ADDRESS, PWR_MGMT_1, &RegReset);

    }while (RegReset & (1 << 7)); {

        // Wait for the bit to clear
    };
	#endif


							// wait untill reset is complete

	writeI2C(MPU_ADDRESS, PWR_MGMT_1, (1 << 3) || 0x03 );		// power managment setup, temp sensor OFF, sleep mode OFF ...
	writeI2C(MPU_ADDRESS, SMPRT_DIV, 0x01);						// sample rate 1kHz
	writeI2C(MPU_ADDRESS, CONFIG, 0x03);						// disable FSYNC, 41 Hz gyro filtering, 1 kHz sampling		??????????
	writeI2C(MPU_ADDRESS, GYRO_CONFIG, (3 << 3));				// gyro full scale range --> 2000 deg/s (3 << 3)
	writeI2C(MPU_ADDRESS, ACC_CONFIG, (2 << 3));				// acc full scale range  --> 8g (2 << 3)

	writeI2C(MPU_ADDRESS, INT_PIN_CFG, 0x30); 	// Configure INT pin or 0011 0000 ??? 0x30
	writeI2C(MPU_ADDRESS, INT_ENABLE, 0x01);	// Enable interrupt DATA READY bit


	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlDelay(3);
	GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_2); // Set as input
//	GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_2, GPIO_RISING_EDGE);
//	GPIOIntRegister(GPIO_PORTE_BASE, readMPU);
//	IntPrioritySet(INT_GPIOE, 0);
//	GPIOIntEnable(GPIO_PORTE_BASE, GPIO_INT_PIN_2);

}

float getAccZ(){
	return accZout; //    /4096 acc sensitivity?
}


float getMPUangleX(){
//	UARTprintf("kot: %d \n", (int)fiXos*57);
	return (fiXos[0]*57.29577951); //
}
float getMPUangleY(){
	return(fiYos[0]*(57.29577951));
}

float getGyroX(){
	return (gyroXos[0]*57.29577951);
}

float getGyroY(){
	return (gyroYos[0]*57.29577951);
}

float getGyroZ(){
	return gyroZos;
}
int dataReadyMPU(){
	int value = GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_2);
	return value;
}

void calibrateMPU(){
	double Ax_off, Ay_off, Az_off;
	int a =0;
	while(a < 1000){
		if(dataReadyMPU()){
				Ax_off += accXout;
				Ay_off += accYout;
				Az_off += accZout;
				a++;
		}
	}


		Ax_off = Ax_off / 1000;
		Ay_off = Ay_off / 1000;
		Az_off = Az_off / 1000;
		UARTprintf("ax  %d,  ay  %d,  az  %d \n", (int)(Ax_off*1000), (int)(Ay_off*1000), (int)(Az_off*1000));
		while(1);

}
