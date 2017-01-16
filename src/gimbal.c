#include <ppm.h>
#include <stdint.h>
#include <stdbool.h>
#include "stdlib.h"
#include "inc/hw_ints.h"
#include "inc/hw_gpio.h"
#include "uart.h"
#include "mpu.h"
#include "gimbal.h"
#include "driverlib/timer.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include <string.h>

#define ANGLEX 1
#define ANGLEY 0

void useGimbal(){
  #if ANGLEX
  float angle = getMPUangleX();
  #endif
  
  #if ANGLEY
  float angle = getMPUangleY();
  #endif

  float error = 0 - angle;
  float gimbalWrite = 1 * error;
  writeGimbal(gimbalWrite);
}
