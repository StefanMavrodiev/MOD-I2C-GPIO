/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

#if defined(__XC)
    #include <xc.h>         /* XC8 General Include File */
#elif defined(HI_TECH_C)
    #include <htc.h>        /* HiTech General Include File */
#endif

#include <stdint.h>        /* For uint8_t definition */
#include <stdbool.h>       /* For true/false definition */
#include <string.h>

#include "registers.h"
#include "system.h"        /* System funct/params, like osc/peripheral config */
#include "user.h"          /* User funct/params, such as InitApp */

/******************************************************************************/
/* User Global Variable Declaration                                           */
/******************************************************************************/

/**
 * @brief Make pointers.
 * 
 * A pointer used for I2C communication. On each read/write address will
 * increase. If requested register is outside regmap, the pointer will 
 * retrun dummy byte.
 */
struct registers regmap, current_regmap;

uint8_t *pointer = (uint8_t *)&regmap;

eeprom uint32_t serial = 0x12345678;
enum RequestType req;



/******************************************************************************/
/* Main Program                                                               */
/******************************************************************************/
void main(void)
{
    /* Configure the oscillator for the device */
    ConfigureOscillator();

    /* Configure default values */
    regmap.device = DEVICE_ID;
    regmap.firmware = FIRMWARE_VERSION;
    regmap.serial = serial;
    regmap.function = 0x00;                 /* All GPIOs */
    regmap.dir = 0xFF;                      /* All inputs */
    regmap.input = 0x00;                    /* Input */
    regmap.output = 0x00;                   /* Output */
    regmap.pullup = 0xFF;                   /* All pull-ups enabled */
    regmap.mode = 0x00;                     /* All push-pull mode */
    regmap.buffer = 0xFF;                   /* All ST enabled inputs */
    regmap.slew = 0xFF;                     /* Slew rate is limited */
 
    regmap.interrupt_enable = 0x00;         /* All interrupts are disabled */
    regmap.interrupt_sense = 0x0000;
    regmap.interrupt_status = 0x00;

    /* Initialize I/O and Peripherals for application */
    InitApp();    
    
    /* Copy default values to current settings */
    memcpy(&current_regmap, &regmap, sizeof(struct registers));

    while(1)
    {
        /* Check for change in registers */
        if(memcmp(&regmap, &current_regmap, sizeof(struct registers))) {
            
            /* Interrupts must be disabled during pins setting */          
//            INTCONbits.GIE = 0;
            /* Disabled SSPI interrupts */
            PIE1bits.SSP1IE = 0;
            
            /* Check for direction changes */
            if(regmap.dir != current_regmap.dir) {
                SetGPIODirection();
                current_regmap.dir = regmap.dir;
            }
            
            if(regmap.output != current_regmap.output) {
                SetGPIOOutput();
                current_regmap.output = regmap.output;
            }
            
            if(regmap.pullup != current_regmap.pullup) {
                SetGPIOPullUp();
                current_regmap.pullup = regmap.pullup;
            }
            
            if(regmap.mode != current_regmap.mode) {
                SetGPIOMode();
                current_regmap.mode = regmap.mode;
            }
            
            if(regmap.buffer != current_regmap.buffer) {
                SetGPIOBuffer();
                current_regmap.buffer = regmap.buffer;
            }
            
            if(regmap.slew != current_regmap.slew) {
                SetGPIOSlew();
                current_regmap.slew = regmap.slew;
            }            
             
            PIE1bits.SSP1IE = 1;
//             INTCONbits.GIE = 1;
        }
        
        CLRWDT();
    }

}

