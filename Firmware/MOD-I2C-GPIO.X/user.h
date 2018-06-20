/******************************************************************************/
/* User Level #define Macros                                                  */
/******************************************************************************/

#define DEVICE_ID           0x43
#define FIRMWARE_VERSION    0x01

#define I2C_SLAVE_ADDRESS   0x3B
#define I2C_SLAVE_MASK      0x7F

#define INT_ASSERT()        LATAbits.LATA5 = 0
#define INT_DEASSERT()      LATAbits.LATA5 = 1   

/******************************************************************************/
/* User Function Prototypes                                                   */
/******************************************************************************/
enum RequestType {
    I2C_NEXT_IS_ADDR,
    I2C_NEXT_IS_DATA,
};

void InitApp(void);         /* I/O and Peripheral Initialization */

void SetGPIODirection(void);

void SetGPIOOutput(void);
void GetGPIOInput(void);

void SetGPIOPullUp(void);
void SetGPIOMode(void);
void SetGPIOBuffer(void);
void SetGPIOSlew(void);

extern enum RequestType req;