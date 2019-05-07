/******************************************************************************/
/* User Level #define Macros                                                  */
/******************************************************************************/

#define I2C_SLAVE_ADDRESS   0x3B
#define I2C_SLAVE_MASK      0x7F

#define FLASH_END_ADDR      0x1000
#define FLASH_START_ADDR    0x200

/******************************************************************************/
/* User Function Prototypes                                                   */
/******************************************************************************/

void InitApp(void);         /* I/O and Peripheral Initialization */
uint16_t AppChecksum(void);
uint16_t StoredChecksum(void);
