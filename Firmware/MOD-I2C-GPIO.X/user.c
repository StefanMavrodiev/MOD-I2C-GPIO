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
#include <pic16lf18324.h>        

#include "user.h"
#include "registers.h"

/******************************************************************************/
/* User Functions                                                             */
/******************************************************************************/

static void __InitGPIO(void);
static void __InitMSSP(void);
static void __InitInterrupts(void);



/**
 * @brief Translate regmap to register
 * 
 * @param reg   Pointer to SFR
 * @param data  Pointer to regmap
 * @param mask  Mask, 0 if unused
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
 */
static void __SetRegister(volatile uint8_t *reg, uint8_t *data, uint8_t mask)
{
    uint8_t temp;
    
    /* Disable interrupts */
    INTCONbits.GIE = 0;
    
    /* Apply mask */
    *data &= mask;
    
    /* Configure SFR */
    temp = *data & 0x07 | ((*data & 0x08) << 1);
    *reg &= ~((*reg ^ temp) & 0x17);
    *reg |= temp;
    
    /* Configure SFR + 2 */
    reg += 2;
    temp = (*data & 0xF0) >> 2;
    
    *reg &= ~((*reg ^ temp) & 0x3C);
    *reg |= temp;
}

/**
 * @brief Translate SFR -> regmap
 * 
 * @param reg   Pointer to SFR register
 * @param data  Pointer to remap
 * @param mask  Mask, 0 if unused
 */
static void __GetRegister(volatile uint8_t *reg, uint8_t *data, uint8_t mask)
{
    uint8_t temp;
    
    /* Read PORTA */
    temp = (*reg & 0x07) | ((*reg & 0x10) >> 1);
    
    /* Read PORTC */
    temp |= (*reg & 0x3C) << 2;
    
    *data &= ~mask;
    *data |= temp & mask;    
}


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
    
    /* Make OD */
    ODCONAbits.ODCA5 = 1;
    
    /* Configure value */
    LATAbits.LATA5 = 0;
    
    SetGPIODirection();
    SetGPIOData();
    SetGPIOMode();
    
    
    SetGPIOPullUp();
    SetGPIOBuffer();    
    SetGPIOSlew();
    
    GetGPIOData();
    
    /* Enable IOC on both edges */
    IOCAP = 0x17;
    IOCAN = 0x17;
    IOCAF = 0;
    
    IOCCP = 0x3C;
    IOCCN = 0x3C;
    IOCCF = 0;
    
}

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
    
    /* Enable IOC interrupts */
    PIR0bits.IOCIF = 0;
    PIE0bits.IOCIE = 1;
    
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
 */
void SetGPIODirection(void)
{    
    __SetRegister(&TRISA, &regmap.dir, -1);
}

/**
 * @brief Set output pin value
 * 
 * Set value of the pins configured as outputs. Values are read from the
 * global register, so this should be called only when there is change in it.
 * If pin is configured as input, this function do nothing.
 * @see SetGPIODirection
 */
void SetGPIOData(void)
{
    __SetRegister(&LATA, &regmap.data, ~regmap.dir);
}

/**
 * @brief Read input pins
 * 
 * Read current value of the pins configured as inputs and stores them into
 * the global register.
 */
void GetGPIOData(void)
{
    __GetRegister(&PORTA, &regmap.data, regmap.dir);
}

/**
 * @brief Sets pull-ups
 * 
 * Enable/disable internal weak pull-ups. If pins is configured as output
 * setting this value has no effect, except for OD configuration.
 */
void SetGPIOPullUp(void)
{
    __SetRegister(&WPUA, &regmap.pullup, -1);
}

/**
 * @brief Enabled open-drain configuration
 * 
 * Enable/disable open-drain output configuration.
 */
void SetGPIOMode(void)
{
    __SetRegister(&ODCONA, &regmap.mode, -1);
}

/**
 * @brief Select CMOS/TTL input level
 * 
 */
void SetGPIOBuffer(void)
{
    __SetRegister(&INLVLA, &regmap.buffer, -1);
}

/**
 * @brief Select input slew rate
 * 
 * Limit or allow maximum slew rate.
 */
void SetGPIOSlew(void)
{
    __SetRegister(&SLRCONA, &regmap.buffer, -1);
}