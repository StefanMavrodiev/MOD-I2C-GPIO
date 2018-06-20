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

enum commands {
    CMD_READ_VERSION = 0,
    CMD_READ_FLASH,
    CMD_WRITE_FLASH,
    CMD_ERASE_FLASH,
    CMD_READ_EE_DATA,
    CMD_WRITE_EE_DATA,
    CMD_READ_CONFIG,
    CMD_WRITE_CONFIG,
    CMD_CALC_CHECKSUM,
    CMD_RESET_DEVICE
};

uint8_t buffer[64];

/******************************************************************************/
/* Main Program                                                               */
/******************************************************************************/
void main(void)
{
    /* Configure the oscillator for the device */
    ConfigureOscillator();

    /* Initialize I/O and Peripherals for application */
    InitApp();
    
    
    /* If RA5 is high load application */
    // TODO: Checksum must be also read
    
    if(PORTAbits.RA5) {
        
        STKPTR = 0x1F;
        asm ("pagesel 0x200");
        asm ("goto 0x200");
    }

    while(1)
    {
    }

}

