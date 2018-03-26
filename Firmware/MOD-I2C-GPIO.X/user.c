/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

#if defined(__XC)
    #include <xc.h>         /* XC8 General Include File */
#elif defined(HI_TECH_C)
    #include <htc.h>        /* HiTech General Include File */
#endif

#include <stdint.h>         /* For uint8_t definition */
#include <stdbool.h>
#include <pic16lf18324.h>        /* For true/false definition */

#include "user.h"
#include "registers.h"

/******************************************************************************/
/* User Functions                                                             */
/******************************************************************************/

static void __InitGPIO(void);
static void __InitMSSP(void);
static void __InitInterrupts(void);

static inline void __SetGPIODirection(void);
static inline void __SetGPIOData(void);
static inline void __GetGPIOData(void);

void InitApp(void)
{
    __InitGPIO();
    __InitMSSP();
    __InitInterrupts();
}

/**
 * @brief Initialize GPIO
 * 
 * Setup GPIO as follows:
 *  - Analog functionality is disabled
 *  - RC0 and RC1 as I2C functions
 *  - RA5 open-drain output
 *  - All other pins as I/O
 */
static void __InitGPIO(void)
{
    /* Disable analog functions */
    ANSELA = 0x00;
    ANSELC = 0x00;
    
    /* Configure PPS function */
    RC0PPS = 0x18;
    RC1PPS = 0x19;
    
    /* Configure direction */
    TRISAbits.TRISA5 = 0;
    __SetGPIODirection();
    
    /* Configure value */
    
}

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

/**
 * @brief Configure interrupts
 * 
 * Enable individual peripheral interrupts.
 */
static void __InitInterrupts(void)
{
    /* Enable MSSP interrupts */
    PIR1bits.SSP1IF = 0;
    PIE1bits.SSP1IE = 1;    
    
    /* Enable Peripheral interrupts */
    INTCONbits.PEIE = 1;
    
    /* Enable Global interrupts */
    INTCONbits.GIE = 1;
    
}

/**
 * @brief Set pin direction
 * 
 * Set direction of GPIO0 to GPIO7. Data is read from the global resister,
 * so this should be called only when there is change in it.
 * Pin direction is changed only if needed.
 * 
 * The algorithm is:
 *  1.  The input data is e.g: 0xAB.
 *      The upper four bit are for PORTC and the lower 4 for PORTA.
 * 
 *  2.  Bytes for PORTA are padded to match pin register:
 *      | x | x | x | x | 3 | 2 | 1 | 0 |
 *      ---------------------------------
 *      | x | x | x | 3 | x | 2 | 1 | 0 |
 * 
 *  3.  Current direction is XORed with the new value. Then only bits with
 *      change are cleared.
 *      | x | x | x | 1 | x | 1 | 0 | 0 |   -> original value
 *      | x | x | x | 1 | x | 0 | 1 | 0 |   -> desired value
 *      ---------------------------------   
 *      | x | x | x | 0 | x | 1 | 1 | 0 |   -> XOR
 *      ---------------------------------
 *      | x | x | x | 1 | x | 0 | 0 | 1 |   -> NOT
 * 
 *      | x | x | x | 1 | x | 1 | 0 | 0 |   -> original value
 *      | x | x | x | 1 | x | 0 | 0 | 1 |   -> NOT value
 *      ---------------------------------   
 *      | x | x | x | 1 | x | 0 | 0 | 0 |   -> AND
 * 
 *  4.  Finally cleared value is OR-ed with the desired one"
 *      | x | x | x | 1 | x | 0 | 0 | 0 |   -> AND value
 *      | x | x | x | 1 | x | 0 | 1 | 0 |   -> desired value
 *      --------------------------------
 *      | x | x | x | 1 | x | 0 | 1 | 0 |   -> final value
 * 
 * This is a bit complicated, but this way we can guarantee that direction
 * will not toggle if there is no change in direction register.
 *      
 */
static inline void __SetGPIODirection(void)
{
    uint8_t isr = INTCON;
    uint8_t dir;
    
    /* Disable interrupts */
    INTCONbits.GIE = 0;
    
    /* Configure PORTA */
    dir = regmap.dir & 0x07 | ((regmap.dir & 0x08) << 1);
    
    TRISA &= ~((TRISA ^ dir) & 0x17);
    TRISA |= dir;
    
    /* Configure PORTC */
    dir = (regmap.dir & 0xF0) >> 2;
    
    TRISC &= ~((TRISC ^ dir) & 0x3C);
    TRISC |= dir;
    
    /* Restore interrupts */
    INTCON = isr;
}

/**
 * @brief Set output pin value
 * 
 * Set value of the pins configured as outputs. Values are read from the
 * global register, so this should be called only when there is change in it.
 * If pin is configured as input, this function do nothing.
 * @see __SetGPIODirection
 */
static inline void __SetGPIOData(void)
{
    uint8_t isr = INTCON;
    uint8_t data;
    
    /* Disable interrupts */
    INTCONbits.GIE = 0;
    
    /* Change only output */
    regmap.data &= regmap.dir;
    
    /* Configure PORTA */
    data = regmap.data & 0x07 | ((regmap.data & 0x08) << 1);
    
    LATA &= ~((LATA ^ data) & 0x17);
    LATA |= data;
    
    /* Configure PORTC */
    data = (regmap.data & 0xF0) >> 2;
    
    
    LATC &= ~((LATC ^ data) & 0x3C);
    LATC |= data;
    
    /* Restore interrupts */
    INTCON = isr;
}

/**
 * @brief Read input pins
 * 
 * Read current value of the pins configured as inputs and stores them into
 * the global register.
 */
static inline void __GetGPIOData(void)
{
    uint8_t isr = INTCON;
    uint8_t data;
    
    /* Disable interrupts */
    INTCONbits.GIE = 0;
    
    /* Read PORTA */
    data = (PORTA & 0x07) | ((PORTA & 0x10) >> 1);
    
    /* Read PORTC */
    data |= (PORTC & 0x3C) << 2;
    
    /* Store new values */
    regmap.data &= ~regmap.dir;
    regmap.data |= data & regmap.dir;   
            
    /* Restore interrupts */
    INTCON = isr;
}
