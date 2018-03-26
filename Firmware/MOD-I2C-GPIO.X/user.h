/******************************************************************************/
/* User Level #define Macros                                                  */
/******************************************************************************/

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

extern enum RequestType req;