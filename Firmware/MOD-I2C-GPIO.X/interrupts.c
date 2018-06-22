/******************************************************************************/
/*Files to Include                                                            */
/******************************************************************************/

#if defined(__XC)
    #include <xc.h>         /* XC8 General Include File */
#elif defined(HI_TECH_C)
    #include <htc.h>        /* HiTech General Include File */
#endif

#include <stdint.h>         /* For uint8_t definition */
#include <stdbool.h>        /* For true/false definition */
#include <pic16lf18324.h>

#include "registers.h"
#include "user.h"

/******************************************************************************/
/* Interrupt Routines                                                         */
/******************************************************************************/

static inline void __MSSPInterrupt(void);
static inline void __IOCInterrupt(void);

void interrupt isr(void)
{
    if(PIR1bits.SSP1IF)
        __MSSPInterrupt();
    else if(PIR0bits.IOCIF)
        __IOCInterrupt();
}

static inline void __IOCInterrupt(void)
{
    volatile uint8_t ioca;
    volatile uint8_t iocc;
    volatile uint8_t data;
    volatile uint8_t ioc;
    volatile uint8_t mask;
    volatile uint8_t i;    
    
    /* Clear interrupt flag */
    PIR0bits.IOCIF = 0;
    
    /* Store current interrupt flags */
    ioca = IOCAF;
    iocc = IOCCF;
    
    /* Read pin inputs */
    data = (PORTA & 0x07) | ((PORTA & 0x10) >> 1);
    data |= (PORTC & 0x3C) << 2;
    
    /* Mask data */
    regmap.input = data;
    
    /* Map IOCxF to regmap */
    ioc = (ioca & 0x07) | ((ioca & 0x10) >> 1);
    ioc |= (iocc & 0x3C) << 2;
    
    /* Check for INT request */
    mask = ioc & regmap.interrupt_enable;
    if (mask) {
        if (regmap.input & mask) {
            
            /**
             * Rising edge
             * Check if sense is correct
             */
            for (i = 0; i < 8; i++) {
                if (mask & (1 << i)) {
                    if (regmap.interrupt_sense & (0x01 << i*2)) {
                        regmap.interrupt_status |= mask;
                        INT_ASSERT();
                    }
                }
            }
            
        } else {
            
            /**
             * Falling edge
             * Check if sense is correct
             */
            for(i = 0; i < 8; i++) {
                if(mask & (1 << i)) {
                    if(regmap.interrupt_sense & (0x02 << i*2)) {
                        regmap.interrupt_status |= mask;
                        INT_ASSERT();
                    }
                }                
            }            
        }
    }
    
    /* Clear IOC flags */
    IOCAF &= ~(ioca);
    IOCCF &= ~(iocc);           
}
/**
 * @brief Handle MSSP interrupts
 */
static inline void __MSSPInterrupt(void)
{
    volatile uint8_t data;
    
    /* Clear interrupt flag */
    PIR1bits.SSP1IF = 0;
    
    /* Clear BF */
    data = SSP1BUF;    
    
    if(SSP1STATbits.R_nW)
    {
        /**
         * We've got READ from MASTER
         */
        if((SSP1STATbits.D_nA) && (SSP1CON2bits.ACKSTAT))
        {
            if (pointer >= (uint8_t *)&regmap.interrupt_status)
                pointer = (uint8_t *)&regmap;
            else
                ++pointer;
        }
        else
        {
            /* Got read request */
            
            SSPBUF = *pointer;
            
            /* Clear pending INT on INTERRUPT_STATUS read */
            if(pointer == (uint8_t *)&regmap.interrupt_status) {
                regmap.interrupt_status = 0x00;
                INT_DEASSERT();
            }
            
            ++pointer;
                
        }
    }
    else if(!SSP1STATbits.D_nA)
    {
        /**
         * Got write request.
         * Next will be registry address
         */
        req = I2C_NEXT_IS_ADDR;
    }
    
    else
    {
        /* Process data */
        if(req == I2C_NEXT_IS_ADDR) {
            if(data < sizeof(struct registers))
            pointer = (uint8_t *)&regmap + data;
        } else {
            if(pointer >= &regmap.dir &&
                    pointer != &regmap.input &&
                    pointer != &regmap.interrupt_status)
                *pointer = data;
        }
        
        req = I2C_NEXT_IS_DATA;        
        
    }

    /* Release SCL */
    SSP1CON1bits.CKP = 1;    
}
