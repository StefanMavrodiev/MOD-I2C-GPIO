/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

#if defined(__XC)
    #include <xc.h>         /* XC8 General Include File */
#elif defined(HI_TECH_C)
    #include <htc.h>        /* HiTech General Include File */
#endif

#include <stdint.h>        /* For uint8_t definition */
#include <stdbool.h>
#include <pic16lf18324.h>       /* For true/false definition */

#include "system.h"        /* System funct/params, like osc/peripheral config */
#include "user.h"          /* User funct/params, such as InitApp */

/******************************************************************************/
/* User Global Variable Declaration                                           */
/******************************************************************************/

enum RequestType {
    I2C_NEXT_IS_ADDRL,
    I2C_NEXT_IS_ADDRH,
    I2C_NEXT_IS_DATA,
} req;

volatile uint8_t id = 0x42;

/******************************************************************************/
/* Main Program                                                               */
/******************************************************************************/
void main(void)
{
    volatile uint8_t data;
    uint16_t checksum[2];
    union {
            uint16_t addr;
            struct {
                uint8_t addrl;
                uint8_t addrh;
            };
            
    } pointer;
    
    /* Configure the oscillator for the device */
    ConfigureOscillator();

    /* Initialize I/O and Peripherals for application */
    InitApp();
    
    checksum[0] = AppChecksum();
    checksum[1] = StoredChecksum();
    
    
    /* If RA5 is high load application */    
    if(PORTAbits.RA5 && (checksum[0] == checksum[1])) {
        
        STKPTR = 0x1F;
        asm ("pagesel 0x200");
        asm ("goto 0x200");
    }

    /* Start waiting for  i2c packets */
    while(1)
    {
        /* Waif for BF flag */
        if (!PIR1bits.SSP1IF)
            continue;
        
        /* Clear interrupt flag */
        PIR1bits.SSP1IF = 0;
    
        /* Clear BF */
        data = SSP1BUF;    
    
        if(SSP1STATbits.R_nW) {
            /**
             * We've got READ from MASTER
             */
            if((SSP1STATbits.D_nA) && (SSP1CON2bits.ACKSTAT)) {
                    ++pointer;
            } else {
                /* Got read request */

                SSPBUF = *pointer;
                ++pointer;  
            }
        } else if(!SSP1STATbits.D_nA) {
            /**
             * Got write request.
             * Next will be registry address
             */
            req = I2C_NEXT_IS_ADDRL;
        } else {
            /* Process data */
            if (req == I2C_NEXT_IS_ADDRL) {
                pointer.addrl = data;
                req = I2C_NEXT_IS_ADDRH;
            } else if (req == I2C_NEXT_IS_ADDRH) {
                pointer.addrh = data;
            } else {
                if(pointer >= &regmap.dir &&
                        pointer != &regmap.input &&
                        pointer != &regmap.interrupt_status)
                    *pointer = data;

                ++pointer;
            }

            req = I2C_NEXT_IS_DATA;        
        }

        /* Release SCL */
        SSP1CON1bits.CKP = 1;   

        }
}

