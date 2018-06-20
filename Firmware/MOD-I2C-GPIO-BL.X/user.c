/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

#if defined(__XC)
    #include <xc.h>         /* XC8 General Include File */
#elif defined(HI_TECH_C)
    #include <htc.h>        /* HiTech General Include File */
#endif

#include <stdint.h>         /* For uint8_t definition */
#include <stdbool.h>        /* For true/false definition */

#include "user.h"

/******************************************************************************/
/* User Functions                                                             */
/******************************************************************************/

/**
 * @brief Initialize I2C controller
 * 
 * Initialize MSSP as I2C slave device. It should be capable of 400kHz
 * operation.
 * 
 * @see I2C_SLAVE_ADDRESS
 */
static void __InitMSSP(void)
{
    /**
     * Configure SSP1 STATUS REGISTER as follows:
     *  - SMP: SPI Data Input Sample bit
     *      0 = slew rate control enabled for High-Speed mode (400 kHz)
     *  - CKE: SPI Clock Edge Select bit
     *      1 = Enable input logic so that thresholds are compliant with SMBus specification
     */
    SSP1STAT = 0x40;
    
    /**
     * Configure SSP1CON1: SSP1 CONTROL REGISTER 1 as follows:
     *  - WCOL: Write Collision Detect bit (Transmit mode only)
     *      0 = No collision
     *  - SSPOV: Receive Overflow Indicator bit
     *      0 = No overflow
     *  - SSPEN: Synchronous Serial Port Enable bit
     *      1 = Enables the serial port and configures the SDA and SCL pins as the source of the serial port pins
     *  - CKP: Clock Polarity Select bit
     *      1 = Enable clock
     *  - SSPM: Synchronous Serial Port Mode Select bits
     *      0110 = I2C Slave mode, 7-bit address
     */
    SSP1CON1 = 0x36;
    
    /**
     * Configure SSP1CON2: SSP1 CONTROL REGISTER 2 as follows:
     *  - GCEN: General Call Enable bit
     *      0 = General call address disabled
     *  - SEN: Start Condition Enable/Stretch Enable bit
     *      1 = Clock stretching is enabled for both slave transmit and slave receive (stretch enabled)
     */
    SSP1CON2 = 0x01;
    
    /**
     * Configure SSP1CON3: SSP1 CONTROL REGISTER 3 as follows:
     *  - PCIE: Stop Condition Interrupt Enable bit
     *      0 = Stop detection interrupts are disabled
     *  - SCIE: Start Condition Interrupt Enable bit
     *      0 = Start detection interrupts are disabled
     *  - BOEN: Buffer Overwrite Enable bit
     *      0 = SSPBUF is only updated when SSPOV is clear
     *  - SDAHT: SDA Hold Time Selection bit
     *      0 = Minimum of 100 ns hold time on SDA after the falling edge of SCL
     *  - SBCDE: Slave Mode Bus Collision Detect Enable bit
     *      0 = Slave bus collision interrupts are disabled
     *  - AHEN: Address Hold Enable bit
     *      0 = Address holding is disabled
     *  - DHEN: Data Hold Enable bit
     *      0 = Data holding is disabled
     */
    SSP1CON3 = 0x00;
    
    /* Set ADDRESS and MASK */
    SSP1MSK = (I2C_SLAVE_MASK << 1);
    SSP1ADD = (I2C_SLAVE_ADDRESS << 1);    
    
}

void InitApp(void)
{
    /* Disable ANALOG function */
    ANSELA = 0;
    ANSELC = 0;

}

