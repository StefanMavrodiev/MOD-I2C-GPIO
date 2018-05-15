/******************************************************************************/
/* Main Files to Include                                                      */
/******************************************************************************/

#if defined(__XC)
    #include <xc.h>         /* XC8 General Include File */
#elif defined(HI_TECH_C)
    #include <htc.h>        /* HiTech General Include File */
#endif

// CONFIG1
#pragma config FEXTOSC = OFF    // FEXTOSC External Oscillator mode Selection bits->Oscillator not enabled
#pragma config RSTOSC = HFINT32    // Power-up default value for COSC bits->HFINTOSC with 2x PLL (32MHz)
#pragma config CLKOUTEN = OFF    // Clock Out Enable bit->CLKOUT function is disabled; I/O or oscillator function on OSC2
#pragma config CSWEN = ON    // Clock Switch Enable bit->Writing to NOSC and NDIV is allowed
#pragma config FCMEN = ON    // Fail-Safe Clock Monitor Enable->Fail-Safe Clock Monitor is enabled

// CONFIG2
#pragma config MCLRE = OFF    // Master Clear Enable bit->MCLR/VPP pin function is digital input; MCLR internally disabled; Weak pull-up under control of port pin's WPU control bit.
#pragma config PWRTE = ON    // Power-up Timer Enable bit->PWRT enabled
#pragma config WDTE = SLEEP    // Watchdog Timer Enable bits->WDT enabled while running and disabled in SLEEP/IDLE; SWDTEN is ignored
#pragma config LPBOREN = OFF    // Low-power BOR enable bit->ULPBOR disabled
#pragma config BOREN = ON    // Brown-out Reset Enable bits->Brown-out Reset enabled, SBOREN bit ignored
#pragma config BORV = LOW    // Brown-out Reset Voltage selection bit->Brown-out voltage (Vbor) set to 2.45V
#pragma config PPS1WAY = ON    // PPSLOCK bit One-Way Set Enable bit->The PPSLOCK bit can be cleared and set only once; PPS registers remain locked after one clear/set cycle
#pragma config STVREN = ON    // Stack Overflow/Underflow Reset Enable bit->Stack Overflow or Underflow will cause a Reset
#pragma config DEBUG = OFF    // Debugger enable bit->Background debugger disabled

// CONFIG3
#pragma config WRT = BOOT    // User NVM self-write protection bits->Write protection off
#pragma config LVP = OFF    // Low Voltage Programming Enable bit->High Voltage on MCLR/VPP must be used for programming.

// CONFIG4
#pragma config CP = ON    // User NVM Program Memory Code Protection bit->User NVM code protection enabled
#pragma config CPD = ON    // Data NVM Memory Code Protection bit->Data NVM code protection enabled