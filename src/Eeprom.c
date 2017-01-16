#include "driverlib/eeprom.h"
#include "uart.h"
#include "utils/uartstdio.c"
#include <stdint.h>
#include <stdbool.h>
#include "stdlib.h"
#include "Eeprom.h"

void initEeprom(){
    SysCtlPeripheralEnable(SYSCRL_PERIPH_EEPROM0);
    EEPROMInit();
}

int getEepromSize(){
  int size = EEPROMSizeGet();
  UARTprintf("%d \n", size);
}
int getEepromBlockCount(){
  int block = EEPROMBlockCountGet();
  UARTprintf("%d \n", block);
}
