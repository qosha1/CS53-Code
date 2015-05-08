
/*********************************************************************
File    : i2c.h
Purpose : 
**********************************************************************/
#ifndef __I2C_H__
#define __I2C_H__
/****************************** Includes *****************************/
/****************************** Defines *******************************/
#define SENSORS_I2C               I2C2

#define I2C_SPEED                 400000
#define I2C_OWN_ADDRESS           0x00
#define MPU_I2C_ADDRESS						0x68

#define I2C_Config() I2cMaster_Init();

void I2cMaster_Init(void);
void Set_I2C_Retry(unsigned short ml_sec);
unsigned short Get_I2C_Retry();

int Sensors_I2C_ReadRegister(unsigned char Address, unsigned char RegisterAddr, 
                                          unsigned short RegisterLen, unsigned char *RegisterValue);
int Sensors_I2C_WriteRegister(unsigned char Address, unsigned char RegisterAddr, 
                                           unsigned short RegisterLen, const unsigned char *RegisterValue);
 
#endif // __I2C_H__

#define Delay mdelay

 
#define I2Cx_FLAG_TIMEOUT             ((uint32_t) 900) //0x1100
#define I2Cx_LONG_TIMEOUT             ((uint32_t) (300 * I2Cx_FLAG_TIMEOUT)) //was300
 

#define SENSORS_I2C_SCL_GPIO_PORT         GPIOB
#define SENSORS_I2C_SCL_GPIO_CLK          RCC_AHB1Periph_GPIOB
 
#define SENSORS_I2C_SDA_GPIO_PORT         GPIOB
#define SENSORS_I2C_SDA_GPIO_CLK          RCC_AHB1Periph_GPIOB

#define SENSORS_I2C_RCC_CLK               RCC_APB1Periph_I2C2
#define SENSORS_I2C_AF                    GPIO_AF_I2C2


