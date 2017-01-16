
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
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);
	SysCtlDelay(3);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlDelay(3);

    GPIOPinConfigure(GPIO_PE4_I2C2SCL);
	GPIOPinConfigure(GPIO_PE5_I2C2SDA);

	GPIOPinTypeI2CSCL(GPIO_PORTE_BASE, GPIO_PIN_4);
	GPIOPinTypeI2C(GPIO_PORTE_BASE, GPIO_PIN_5);

	//I2CMasterTimeoutSet(I2C0_BASE, 0xFF);
    I2CMasterInitExpClk(I2C2_BASE, SysCtlClockGet(), true);
    SysCtlDelay(2);
}

bool I2CMasterTimeout(uint32_t ui32Base)
{
    // Return the bus timeout status
    if(HWREG(ui32Base + I2C_O_MCS) & I2C_MCS_CLKTO)
    {
        return(true);
    }
    else
    {
       return(false);
    }
}

void readI2C(uint8_t slave_addr, uint8_t reg, int *data)
{
    I2CMasterSlaveAddrSet(I2C2_BASE, slave_addr, false);
    I2CMasterDataPut(I2C2_BASE, reg);
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while ( I2CMasterBusy(I2C2_BASE));
    I2CMasterSlaveAddrSet(I2C2_BASE, slave_addr, true);
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
    while ( I2CMasterBusy(I2C2_BASE));
    *data = I2CMasterDataGet(I2C2_BASE);
}



// Sends 1 byte over i2c
void writeI2C(uint8_t slave_addr, uint8_t reg, uint8_t data)
{
	I2CMasterSlaveAddrSet(I2C2_BASE, slave_addr, false);
	I2CMasterDataPut(I2C2_BASE, reg);
	I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_START);
	while ( I2CMasterBusy(I2C2_BASE));
	I2CMasterDataPut(I2C2_BASE, data);
	I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
	while ( I2CMasterBusy(I2C2_BASE));
}

