
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"

#include "i2c.h"


void initI2C(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
	SysCtlDelay(3);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlDelay(3);

    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
	GPIOPinConfigure(GPIO_PB3_I2C0SDA);

	GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
	GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), true);
}

void readI2C(uint8_t slave_addr, uint8_t reg, int *data)
{
    I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, false);
    I2CMasterDataPut(I2C0_BASE, reg);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(I2CMasterBusy(I2C0_BASE));
    I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, true);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
    while(I2CMasterBusy(I2C0_BASE));
    *data = I2CMasterDataGet(I2C0_BASE);
}


// Sends 1 byte over i2c
void writeI2C(uint8_t slave_addr, uint8_t reg, uint8_t data)
{
	I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, false);
	I2CMasterDataPut(I2C0_BASE, reg);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
 	while(I2CMasterBusy(I2C0_BASE));
	I2CMasterDataPut(I2C0_BASE, data);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
	while(I2CMasterBusy(I2C0_BASE));
}
