#include <stdint.h>
#include <stdbool.h>
#include "stdlib.h"
#include "driverlib/pin_map.h"

#include "sonar.h"
#include "pid.h"
#include "time.h"
#include "mpu.h"
#include "math.h"
#include "driverlib/gpio.h"

float getSonarActualDistance(){
	float Sonardistance = getGroundSonar();
	float angleX = getMPUangleX();
	float angleY = getMPUangleY();
	float ActualDistance = Sonardistance*cos(angleX)*cos(angleY);
	return ActualDistance;
}


