/* Include file for MCU based i2c constants and functions */

#include <stdint.h>							// 
#include "../Main/mpu_constants.h"							//mpu specific constants 
#include "Queue.h"


#define I2C_SPEED                 400000
#define I2C_OWN_ADDRESS           0x00
#define MPU_I2C_ADDRESS						0xD0

#define MPU_RX_QUEUE_LENGTH			((uint32_t)0x0100)
#define MPU_TX_QUEUE_LENGTH			((uint32_t)0x0020)

 /* i2c control register reset masks */
 
#define I2C_CR2_RESET					((uint32_t)0x03FFFFFF)
#define CR1_CLEAR_MASK				((uint32_t)0x00FFDFFF)

#define MPU_I2C_TIMING_SCLDEL			((uint32_t)0x00000003)//4
#define MPU_I2C_TIMING_SDADEL			((uint32_t)0x00000002)//2
#define MPU_I2C_TIMING_SCLH				((uint32_t)0x00000003)//F
#define MPU_I2C_TIMING_SCLL				((uint32_t)0x00000009)//13
#define MPU_I2C_TIMING_PRESC			((uint32_t)0x00000000)

extern Queue *mpuTxQueue; // ensure only one TxQueue
extern Queue *mpuRxQueue; // ensure only one RxQueue

#ifndef I2C_INITIALIZE
#define I2C_INITIALIZE 

typedef struct _I2C_Timing
{
	uint32_t SCLDEL;
	uint32_t SDADEL;
	uint32_t SCLH;
	uint32_t SCLL;
	uint32_t PRESC;
}I2C_Timing;

typedef struct _I2C_Initstruct
{
	uint8_t PeriphAddress;
	I2C_Timing *ClockSpeed;
	boolean	AutoEnd;
	boolean Reload;
}I2C_Initstruct;


void mpu_I2C_Init(void);
void mpu_writeRegister(uint16_t numBytes, uint16_t reg, boolean writeForRead);
void mpu_readRegister(uint16_t numBytes, uint16_t reg);
boolean mpu_isInitialized();

extern void I2C1_EV_IRQHandler(void);
void Delay (uint32_t dlyTicks);
#endif	/* i2c_initialize */