/******************************************************************************/
/* System Level #define Macros                                                */
/******************************************************************************/

/* Microcontroller MIPs (FCY) */
#define SYS_FREQ        32000000L
#define _XTAL_FREQ      SYS_FREQ
#define FCY             SYS_FREQ/4

/******************************************************************************/
/* System Function Prototypes                                                 */
/******************************************************************************/

void ConfigureOscillator(void); /* Handles clock switching/osc initialization */
