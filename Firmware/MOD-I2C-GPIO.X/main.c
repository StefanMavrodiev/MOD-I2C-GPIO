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
struct registers regmap;

uint8_t *pointer = (uint8_t *)&regmap;

eeprom uint32_t serial = 0x12345678;
struct registers current;
enum RequestType req;

/******************************************************************************/
/* Main Program                                                               */
/******************************************************************************/
void main(void)
{
    /* Configure the oscillator for the device */
#ifndef __SIM__
    ConfigureOscillator();
#endif

    regmap.device = DEVICE_ID;
    regmap.firmware = FIRMWARE_VERSION;
    regmap.serial = serial;
    regmap.dir = 0xFF;                      /* All inputs */
    regmap.input = 0x00;                    /* Input */
    regmap.output = 0x00;                   /* Output */
    regmap.pullup = 0xFF;                   /* All pull-ups enabled */
    regmap.mode = 0x00;                     /* All push-pull mode */
    regmap.buffer = 0xFF;                   /* All ST enabled inputs */
    regmap.slew = 0xFF;                     /* Slew rate is limited */
#ifdef __SIM__
    regmap.interrupt_enable = 0x01;         /* All interrupts are disabled */
    regmap.interrupt_sense = 0x0003;
#else
    regmap.interrupt_enable = 0x00;         /* All interrupts are disabled */
    regmap.interrupt_sense = 0x0000;
#endif
    regmap.interrupt_status = 0x00;

    /* Initialize I/O and Peripherals for application */
    InitApp();    
    
    /* Copy default values to current settings */
    memcpy(&current, &regmap, sizeof(struct registers));

    while(1)
    {
        /* Check for change in registers */
        if(memcmp(&regmap, &current, sizeof(struct registers))) {
            
            /* Interrupts must be disabled during pins setting */          
            INTCONbits.GIE = 0;
            
            /* Check for direction changes */
            if(regmap.dir != current.dir) {
                SetGPIODirection();
                current.dir = regmap.dir;
            }
            
            if(regmap.output != current.output) {
                SetGPIOOutput();
                current.output = regmap.output;
            }
            
            if(regmap.pullup != current.pullup) {
                SetGPIOPullUp();
                current.pullup = regmap.pullup;
            }
            
            if(regmap.mode != current.mode) {
                SetGPIOMode();
                current.mode = regmap.mode;
            }
            
            if(regmap.buffer != current.buffer) {
                SetGPIOBuffer();
                current.buffer = regmap.buffer;
            }
            
            if(regmap.slew != current.slew) {
                SetGPIOSlew();
                current.slew = regmap.slew;
            }            
             
             INTCONbits.GIE = 1;
        }
        
        CLRWDT();
    }

}

