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
void interrupt isr(void)
{
    if(PIR1bits.SSP1IF)
        __MSSPInterrupt();
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
            if (pointer >= (uint8_t *)&regmap + sizeof(regmap))
                pointer = (uint8_t *)&regmap;
            else
                ++pointer;
        }
        else
        {
            /* Got read request */
            SSPBUF = *pointer;
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
            if(pointer != &regmap.device && pointer != &regmap.firmware)
                *pointer = data;
        }
        
        req = I2C_NEXT_IS_DATA;        
        
    }

    /* Release SCL */
    SSP1CON1bits.CKP = 1;    
}
