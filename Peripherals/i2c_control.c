#include "i2c_control.h"	// header for MCU specific i2c constants
#include "stm32f373xc.h" // Device header
#include <stdint.h> 			// 
#include <stdlib.h>


Queue *mpuRxQueue;
Queue *mpuTxQueue;
volatile uint32_t msTicks;                      /* counts 1ms timeTicks       */

// function prototypes
void I2C_Initialize(I2C_Initstruct *I2Cx);
void I2C_GenerateSTART(I2C_TypeDef* I2Cx, boolean Enable);

void mpu_I2C_Init(void){
	I2C_Timing *i2c_speed = malloc(sizeof *i2c_speed);
	I2C_Initstruct *I2C_Init = malloc(sizeof *I2C_Init);
	
	RCC->AHBENR	|= RCC_AHBENR_GPIOBEN; 			/* Enable GPIOB clock*/
	// Enable I2C clocks 
	RCC->APB1ENR   |= RCC_APB1ENR_I2C1EN;  	/* Enable I2C_1 clock*/

	/* Initialize the i2c bus GPIO pins    */
	GPIOB->AFR[0]  |= (((uint32_t) MPU_I2C_AF) << 4 * (MPU_SCL % 8));		// alternate function setup
	GPIOB->AFR[0]  |= (((uint32_t) MPU_I2C_AF) << 4 * (MPU_SDA % 8));		// alternate function setup
	
	GPIOB->OTYPER |= (1UL << 1*MPU_SCL);		//  open drain
	GPIOB->OTYPER |= (1UL << 1*MPU_SDA);		//  open drain
	
	GPIOB->OSPEEDR |= (3UL << 2*MPU_SCL);		//  high speed
	GPIOB->OSPEEDR |= (3UL << 2*MPU_SDA);		//  high speed
	
	GPIOB->MODER	 |= (2UL << 2*MPU_SCL)  ;         /* SET TO ALTERNATE FUNCTION	*/
	GPIOB->MODER 	 |= (2UL << 2*MPU_SDA)  ;         /* SET TO ALTERNATE FUNCTION */
	
	// Interrupt signal
	GPIOB->MODER    &=  ~((3UL << 2*MPU_INTRPT)  );   /* mpu_int is input              */
	GPIOB->PUPDR    &=  ~((3UL << 2*MPU_INTRPT)  );   /* mpu_int is floating         */
	
	RCC->APB2ENR	 		 |= RCC_APB2ENR_SYSCFGEN;				/* Turn on sysconfig clock */
	while(!(RCC->APB2ENR & RCC_APB2ENR_SYSCFGEN)){}
	SYSCFG->EXTICR[1]  |= (1UL << 4); 				//Set MPU_INTRPT as external interrupt 5
	EXTI->IMR					 |= (1UL << MPU_INTRPT);//dont mask interrupt
	EXTI->FTSR				 |= (1UL << MPU_INTRPT);// watch for falling edges
	
  // Set the timing values for the i2c bus
	i2c_speed->SCLDEL = MPU_I2C_TIMING_SCLDEL;
	i2c_speed->SDADEL = MPU_I2C_TIMING_SDADEL;
	i2c_speed->SCLH = MPU_I2C_TIMING_SCLH;
	i2c_speed->SCLL = MPU_I2C_TIMING_SCLL;
	i2c_speed->PRESC = MPU_I2C_TIMING_PRESC;
	
	// Assign values to init struct
	I2C_Init->ClockSpeed 		= i2c_speed;
	I2C_Init->PeriphAddress = MPU_I2C_ADDRESS;
  I2C_Initialize(I2C_Init);
	
	/* Setup the queue for data transfer */	
	mpuRxQueue = initQueue(MPU_RX_QUEUE_LENGTH);
	mpuTxQueue = initQueue(MPU_TX_QUEUE_LENGTH);
	free(i2c_speed);
	free(I2C_Init);
}
void I2C_Initialize(I2C_Initstruct *I2Cx)
{
  uint32_t tmpreg = 0;

/*---------------------------- MPU_I2C CR2 Configuration ------------------------*/
  /* Get the MPU_I2C CR2 value */
  tmpreg = MPU_I2C->CR2;
  /* Clear configurable bits */
  tmpreg &= (uint32_t)~((uint32_t)I2C_CR2_RESET);
	

  MPU_I2C->CR2 = tmpreg;

/*---------------------------- MPU_I2C CCR Configuration ------------------------*/
  /* Disable the selected I2C peripheral to configure TRISE */
  MPU_I2C->CR1 &= (uint16_t)~((uint16_t)I2C_CR1_PE);
  /* Reset tmpreg value */
  tmpreg = 0;

  /* Configure speed  */
	tmpreg  |= (I2Cx->ClockSpeed->PRESC << 28); // presecale pin clock 
	tmpreg  |= (I2Cx->ClockSpeed->SCLH << 8) | (I2Cx->ClockSpeed->SCLL << 0);
	tmpreg  |= (I2Cx->ClockSpeed->SCLDEL << 20) | (I2Cx->ClockSpeed->SDADEL << 16);   


  /* Write to MPU_I2C CCR */
  MPU_I2C->TIMINGR = tmpreg;
  /* Enable the selected I2C peripheral */
	MPU_I2C->CR1 |= I2C_CR1_PE;

/*---------------------------- MPU_I2C CR1 Configuration ------------------------*/
  /* Get the MPU_I2C CR1 value */
  tmpreg = MPU_I2C->CR1;
  /* Clear ACK, SMBTYPE and  SMBUS bits */
  tmpreg &= CR1_CLEAR_MASK;
  /* Configure MPU_I2C: mode and acknowledgement */
	// enable interrupts
	tmpreg |= I2C_CR1_TXIE | I2C_CR1_RXIE;
	tmpreg |= I2C_CR1_PE;	
  /* Write to I2Cx CR1 */
  MPU_I2C->CR1 = tmpreg;

/*---------------------------- MPU_I2C OAR1 Configuration -----------------------*/
  /* Set MPU_I2C Own Address1 and acknowledged address */
  MPU_I2C->OAR1 |= (I2C_OWN_ADDRESS << 1);
}

void mpu_writeRegister(uint16_t numBytes, uint16_t reg, boolean writeForRead){
	uint32_t interrupt;
	boolean readWrite = false;
	
	while(MPU_I2C->ISR & I2C_ISR_BUSY){}// wait for any previous transfer to complete
	interrupt = MPU_I2C->ISR;
	if(interrupt & I2C_ISR_STOPF){
		MPU_I2C->ICR |= I2C_ICR_STOPCF; //clear stop flag
	}
	while(MPU_I2C->ISR & I2C_ISR_STOPF){}	// wait for flag to clear
		
	if(!queue_isEmpty(mpuTxQueue)){
		uint32_t tmpReg;
		tmpReg = MPU_I2C->CR2;
		tmpReg &= ~(I2C_CR2_RESET);  // clear previous settings
		
		// configure address and end conditions
		if(writeForRead){ // if writing to read, dont autoend
			tmpReg |= ((I2C_CR2_SADD & MPU_I2C_ADDRESS));		
		}else{
			tmpReg |= ((I2C_CR2_SADD & MPU_I2C_ADDRESS)| I2C_CR2_AUTOEND);
		}
		// set direction and number of bytes
		tmpReg |= ((I2C_CR2_RD_WRN & readWrite) | (I2C_CR2_NBYTES & ((numBytes+1) << 16)));
		// configure actual i2c register
		MPU_I2C->CR2 = tmpReg;
		
		/* Once configured, generate start condition */
		I2C_GenerateSTART(MPU_I2C, true);
		
	}
	
}
void mpu_readRegister(uint16_t numBytes, uint16_t reg){
	uint32_t interrupt =  MPU_I2C->ISR;
	uint32_t tmpReg;	
	
while(MPU_I2C->ISR & I2C_ISR_BUSY){}// wait for any previous transfer to complete
		
	if(interrupt & I2C_ISR_STOPF){
		MPU_I2C->ICR |= I2C_ICR_STOPCF; //clear flags
	}
	enQueue(mpuTxQueue, reg);	// add the register to be read
		
	/* write the register to be read to the mpu */
	mpu_writeRegister(0, reg, true);
	
	while(!(MPU_I2C->ISR & I2C_ISR_TC)){} /* wait for write to complete */
	
	tmpReg = MPU_I2C->CR2;
	tmpReg &= ~(I2C_CR2_RESET);  // 
	
	// configure address and end conditions
	tmpReg |= ((I2C_CR2_SADD & MPU_I2C_ADDRESS)| I2C_CR2_AUTOEND);
	// set direction and number of bytes
	tmpReg |= ((I2C_CR2_RD_WRN) | (I2C_CR2_NBYTES & (numBytes << 16)));
	// configure actual i2c register
	MPU_I2C->CR2 = tmpReg;
	
	/* Once configured, generate a repeat start condition */
	I2C_GenerateSTART(MPU_I2C, true);
	
}


/**
  * @brief  Generates I2Cx communication START condition.
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @param  NewState: new state of the I2C START condition generation.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None.
  */
void I2C_GenerateSTART(I2C_TypeDef* I2Cx, boolean Enable)
{
  if (Enable)
  {
    /* Generate a START condition */
    I2Cx->CR2 |= I2C_CR2_START;
  }
  else
  {
    /* Disable the START condition generation */
    I2Cx->ICR |= ((uint16_t)I2C_ICR_ADDRCF);
  }
}

void I2C1_EV_IRQHandler(void){
	uint32_t interrupt =  MPU_I2C->ISR;
	if(interrupt & I2C_ISR_STOPF){
		MPU_I2C->ICR |= I2C_ICR_STOPCF; //clear flag
	}
	if(!(MPU_I2C->CR2 & I2C_CR2_RD_WRN)){ /* either read OR write */
            /* Writing */
            if(interrupt & I2C_ISR_TC){// transfer complete
		//I2C2->CR2 |= I2C_CR2_STOP; // stop the I2c 
            }else if(interrupt & I2C_ISR_TXE){
                    // TX empty, send next byte
                    MPU_I2C->TXDR = (uint8_t) deQueue(mpuTxQueue);
            }
	}else{
            /* Reading */
		if(interrupt & I2C_ISR_RXNE){
			// RX is not empty, read it before next incoming
			enQueue(mpuRxQueue, MPU_I2C->RXDR);
		}
	}
}

void SysTick_Handler (void) {
	msTicks++;
}
/*----------------------------------------------------------------------------
  delays number of tick Systicks (period set by SysTick_Config)
 *----------------------------------------------------------------------------*/
void Delay (uint32_t dlyTicks) {
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks) { __NOP(); }
}
