/******************************************************************************/
/* User Level #define Macros                                                  */
/******************************************************************************/

#define DEVICE_ID           0x43
#define FIRMWARE_VERSION    0x01

#define I2C_SLAVE_ADDRESS   0x3B
#define I2C_SLAVE_MASK      0x7F

/******************************************************************************/
/* User Function Prototypes                                                   */
/******************************************************************************/
enum RequestType {
    I2C_NEXT_IS_ADDR,
    I2C_NEXT_IS_DATA,
};

void InitApp(void);         /* I/O and Peripheral Initialization */

void SetGPIODirection(void);

void SetGPIOData(void);
void GetGPIOData(void);

void SetGPIOPullUp(void);

extern enum RequestType req;