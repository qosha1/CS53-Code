#include "Display.h"
#include "../font.h"
#include <stdint.h>
#include "stm32f373xc.h"                  // Device header
#include "../Main/mpu_constants.h"				// MPU general constants
#include "i2c_control.h"	// header for MCU specific i2c constants
#include "../mpu_control.h" // communication method
Queue *displayQueue; 
uint8_t display[DISPLAY_HEIGHT][DISPLAY_WIDTH]; // copy of display RAM

// initialize the i2c interface
void display_Init(void){
	uint32_t delay;
	
	RCC->AHBENR	|= RCC_AHBENR_GPIOCEN; /* Enable GPIOC clock*/
	RCC->AHBENR	|= RCC_AHBENR_GPIOFEN; /* Enable GPIOF clock*/
	RCC->APB2ENR	 |= RCC_APB2ENR_SYSCFGEN;
	// Enable I2C clocks 
	RCC->APB1ENR   |= RCC_APB1ENR_I2C2EN;  /* Enable I2C_2 clock*/
	
	
	/*					 Setup the i2c data and clock pins  		*/
	GPIOF->AFR[0]  |= (((uint32_t) DISPLAY_I2C_AF) << 4 * (DISPLAY_SCL % 8));		// alternate function setup
	GPIOF->AFR[0]  |= (((uint32_t) DISPLAY_I2C_AF) << 4 * (DISPLAY_SDA % 8));		// alternate function setup
	
	GPIOF->OTYPER |= (1UL << 1*DISPLAY_SCL);		//  open drain
	GPIOF->OTYPER |= (1UL << 1*DISPLAY_SDA);		//  open drain
	
	GPIOF->OSPEEDR |= (3UL << 2*DISPLAY_SCL);		//  high speed
	GPIOF->OSPEEDR |= (3UL << 2*DISPLAY_SDA);		//  high speed
	
	GPIOF->MODER |= (2UL << 2*DISPLAY_SCL)  ;         /* PF.6 is i2c2 clk*/
	GPIOF->MODER |= (2UL << 2*DISPLAY_SDA)  ;         /* PF.7 is i2c2 data*/
		
	//Display reset output
	GPIOC->MODER    |=  (1UL << 2*DISPLAY_RST)   ;         /* PC.DISPLAY_RST is output             */
  GPIOC->OSPEEDR  &= ~((3UL << 2*DISPLAY_RST)  );         /* PC.DISPLAY_RST is Low Speed          */
  GPIOC->OTYPER   |=  (1UL << 1*DISPLAY_RST);
	GPIOC->PUPDR    |=  (1UL << 2*DISPLAY_RST)   ;         /* PC.DISPLAY_RST is OD, pullup         */
	      
	// Display Rd/Wr output
	GPIOC->MODER    |=  (1UL << 2*DISPLAY_RW)   ;         /* PC.DISPLAY_RW is output             */
  GPIOC->OSPEEDR  &= ~((3UL << 2*DISPLAY_RW)  );         /* PC.DISPLAY_RW is Low Speed          */
  GPIOC->OTYPER   |=  (1UL << 1*DISPLAY_RW);
	GPIOC->PUPDR    &=  ~((3UL << 2*DISPLAY_RW)  ) ;         /* PC.DISPLAY_RW is OD         */
	      
	// display E/RD# output
	GPIOC->MODER    |=  (1UL << 2*DISPLAY_ERD)   ;         /* PC.DISPLAY_ERD is output             */
  GPIOC->OSPEEDR  &= ~((3UL << 2*DISPLAY_ERD)  );         /* PC.DISPLAY_ERD is Low Speed          */
  GPIOC->OTYPER   |=  (1UL << 1*DISPLAY_ERD);
	GPIOC->PUPDR    &=  ~((3UL << 2*DISPLAY_ERD)  ) ;         /* PC.DISPLAY_ERD is OD         */
	        
	// display SA0 Selection
	GPIOC->MODER    |=  (1UL << 2*DISPLAY_DC)   ;         /* PC.DISPLAY_DC is output             */
  GPIOC->OSPEEDR  &= ~((3UL << 2*DISPLAY_DC)  );         /* PC.DISPLAY_DC is Low Speed          */
  GPIOC->OTYPER   |=  (1UL << 1*DISPLAY_DC);
	GPIOC->PUPDR    &=  ~((3UL << 2*DISPLAY_DC)  ) ;         /* PC.DISPLAY_DC is OD         */
	
	EXTI->EMR			 |= (1UL << 23); //dont mask event
	
	
	RCC->APB1RSTR   |= RCC_APB1RSTR_I2C2RST;  /* reset I2C_2 clock*/
	RCC->APB1RSTR   &= ~RCC_APB1RSTR_I2C2RST;  /* release reset I2C_2	clock*/
	
	while(!(RCC->APB1ENR & RCC_APB1ENR_I2C2EN)){
	}
	
	// Set i2c SCL frequency 
	DISPLAY_I2C->TIMINGR  |= (I2C_TIMING_PRESC << 28); // presecale pin clock 
	DISPLAY_I2C->TIMINGR  |= (I2C_TIMING_SCLH << 8) | (I2C_TIMING_SCLL << 0);
	DISPLAY_I2C->TIMINGR  |= (I2C_TIMING_SCLDEL << 20) | (I2C_TIMING_SDADEL << 16);   

	/* Setup the queue for data transfer */	
	displayQueue = initQueue(DISPLAY_QUEUE_LENGTH);
	
	
	/* Cycle through reset */
	
	GPIOC->ODR		  |= (1UL << DISPLAY_RST)  ;		/* PC.DISPLAY_RST go high      */
	
	Delay(DISPLAY_RESET_TIME); /* wait for reset to settle */
	GPIOC->ODR		 &= ~((1UL << DISPLAY_RST)  );		/* PC.DISPLAY_RST go low     */
	
	Delay(DISPLAY_RESET_TIME); /* wait for reset to settle */
	GPIOC->ODR		  |= (1UL << DISPLAY_RST)  ;		/* PC.DISPLAY_RST go high      */
	
	Delay(DISPLAY_RESET_TIME); /* wait for reset to settle */
	
	/* Enqueue display startup sequence */
	enQueue(displayQueue, DISPLAY_WRITE_COMMAND | DISPLAY_WRITE_MULTIPLE);
	enQueue(displayQueue, DISPLAY_OFF);
	enQueue(displayQueue, DISPLAY_WRITE_COMMAND | DISPLAY_WRITE_MULTIPLE);
	enQueue(displayQueue, DISPLAY_SET_CLOCK);
	enQueue(displayQueue, DISPLAY_WRITE_COMMAND | DISPLAY_WRITE_MULTIPLE);
	enQueue(displayQueue, DISPLAY_SET_DIVIDE);
	// multiplex ratio
	enQueue(displayQueue, DISPLAY_WRITE_COMMAND | DISPLAY_WRITE_MULTIPLE);
	enQueue(displayQueue, DISPLAY_SET_MULTIPLEX);
	enQueue(displayQueue, DISPLAY_WRITE_COMMAND | DISPLAY_WRITE_MULTIPLE);
	enQueue(displayQueue, DISPLAY_SET_MULTIPLEX2);
	//display offset
	enQueue(displayQueue, DISPLAY_WRITE_COMMAND | DISPLAY_WRITE_MULTIPLE);
	enQueue(displayQueue, DISPLAY_SET_OFFSET);
	enQueue(displayQueue, DISPLAY_WRITE_COMMAND | DISPLAY_WRITE_MULTIPLE);
	enQueue(displayQueue, DISPLAY_SET_OFFSET1);
	// charge pump
	enQueue(displayQueue, DISPLAY_WRITE_COMMAND | DISPLAY_WRITE_MULTIPLE);
	enQueue(displayQueue, DISPLAY_CHARGE_EN0);
	enQueue(displayQueue, DISPLAY_WRITE_COMMAND | DISPLAY_WRITE_MULTIPLE);
	enQueue(displayQueue, DISPLAY_CHARGE_EN1);
	// start line address
	enQueue(displayQueue, DISPLAY_WRITE_COMMAND | DISPLAY_WRITE_MULTIPLE);
	enQueue(displayQueue, DISPLAY_START_LINE);
	// set normal 
	enQueue(displayQueue, DISPLAY_WRITE_COMMAND | DISPLAY_WRITE_MULTIPLE);
	enQueue(displayQueue, DISPLAY_NORMAL);
	// disable entire on
	enQueue(displayQueue, DISPLAY_WRITE_COMMAND | DISPLAY_WRITE_MULTIPLE);
	enQueue(displayQueue, DISPLAY_ON_NORMAL);
	/*
	// DISPLAY_COM_DIR 64-0
	enQueue(displayQueue, DISPLAY_WRITE_COMMAND | DISPLAY_WRITE_MULTIPLE);
	enQueue(displayQueue, DISPLAY_COM_DIR);
	// Set com pin configuration
	enQueue(displayQueue, DISPLAY_WRITE_COMMAND | DISPLAY_WRITE_MULTIPLE);
	enQueue(displayQueue, DISPLAY_SET_COM_PINS);
	enQueue(displayQueue, DISPLAY_WRITE_COMMAND | DISPLAY_WRITE_MULTIPLE);
	enQueue(displayQueue, DISPLAY_SET_COM_PINS2 | DISPLAY_SET_COM_SEQUENTIAL);
	*/
	enQueue(displayQueue, DISPLAY_WRITE_COMMAND | DISPLAY_WRITE_MULTIPLE);
	enQueue(displayQueue, DISPLAY_PRECHARGE_PERIOD);
	enQueue(displayQueue, DISPLAY_WRITE_COMMAND | DISPLAY_WRITE_MULTIPLE);
	enQueue(displayQueue, DISPLAY_PRECHARGE_PERIOD2);
	//
	enQueue(displayQueue, DISPLAY_WRITE_COMMAND | DISPLAY_WRITE_MULTIPLE);
	enQueue(displayQueue, DISPLAY_ON);
	

	//enQueue(displayQueue, DISPLAY_WRITE_COMMAND  | DISPLAY_WRITE_MULTIPLE);
	//enQueue(displayQueue, DISPLAY_SCROLL_OFF);
	//enQueue(displayQueue, DISPLAY_WRITE_COMMAND | DISPLAY_WRITE_MULTIPLE);
	//enQueue(displayQueue, DISPLAY_START_LINE);
	//enQueue(displayQueue, DISPLAY_WRITE_COMMAND | DISPLAY_WRITE_MULTIPLE);
	//enQueue(displayQueue, DISPLAY_COM_NORMAL);

	
	// send commands to the display
	display_Communicate(STARTUP_LENGTH, 0,0);
	
	while(!queue_isEmpty(displayQueue) && !(DISPLAY_I2C->CR2 & I2C_CR2_START)){
		// wait for the startup sequence to finish
		
	}
	Delay(200);
	
	//Setup controller to have data written to it
	enQueue(displayQueue, DISPLAY_WRITE_COMMAND | DISPLAY_WRITE_MULTIPLE);
	enQueue(displayQueue, SET_MEMADD_MODE);
	enQueue(displayQueue, DISPLAY_WRITE_COMMAND | DISPLAY_WRITE_MULTIPLE);
	enQueue(displayQueue, HORIZ_MEMADD_MODE);
	
	enQueue(displayQueue, DISPLAY_WRITE_COMMAND | DISPLAY_WRITE_MULTIPLE);
	enQueue(displayQueue, SET_COLADD);
	enQueue(displayQueue, DISPLAY_WRITE_COMMAND | DISPLAY_WRITE_MULTIPLE);
	enQueue(displayQueue, DISPLAY_COLADD_START);	
	enQueue(displayQueue, DISPLAY_WRITE_COMMAND | DISPLAY_WRITE_MULTIPLE);
	enQueue(displayQueue, DISPLAY_COLADD_END);
	
	enQueue(displayQueue, DISPLAY_WRITE_COMMAND | DISPLAY_WRITE_MULTIPLE);
	enQueue(displayQueue, SET_PAGE_ADDRESS);
	enQueue(displayQueue, DISPLAY_WRITE_COMMAND | DISPLAY_WRITE_MULTIPLE);
	enQueue(displayQueue, DISPLAY_PAGEADD_START);	
	enQueue(displayQueue, DISPLAY_WRITE_COMMAND | DISPLAY_WRITE_MULTIPLE);
	enQueue(displayQueue, DISPLAY_PAGEADD_END);
	/*
	enQueue(displayQueue, SET_COLADD_START0 | (LOW_NIBBLE & HR_X_START));
  enQueue(displayQueue, DISPLAY_WRITE_COMMAND | DISPLAY_WRITE_MULTIPLE);
	enQueue(displayQueue, SET_COLADD_START1 | ((HIGH_NIBBLE & HR_X_START)>>4));
	enQueue(displayQueue, DISPLAY_WRITE_COMMAND | DISPLAY_WRITE_MULTIPLE);
	
	enQueue(displayQueue, SET_PAGEADD_START | HR_Y_START);
	enQueue(displayQueue, DISPLAY_WRITE_COMMAND | DISPLAY_WRITE_MULTIPLE);
	*/
	display_Communicate(16, 0,0);
	while(!queue_isEmpty(displayQueue)){
		// wait for the startup sequence to finish
	}
	Delay(DISPLAY_RESET_TIME); // wait for display to finish
}

void setup_Draw(uint16_t drawPage, uint16_t drawCol){
	uint32_t delay;
	drawPage &= 0x7;
	enQueue(displayQueue, DISPLAY_WRITE_COMMAND | DISPLAY_WRITE_MULTIPLE);
	enQueue(displayQueue, SET_PAGEADD_START | drawPage);
	enQueue(displayQueue, DISPLAY_WRITE_COMMAND | DISPLAY_WRITE_MULTIPLE);
	enQueue(displayQueue, SET_COLADD_START0 | (LOW_NIBBLE & drawCol));
	enQueue(displayQueue, DISPLAY_WRITE_COMMAND | DISPLAY_WRITE_MULTIPLE);
	enQueue(displayQueue, SET_COLADD_START1 | ((HIGH_NIBBLE & drawCol)>>4));
	display_Communicate(6, 0,0);
	delay = 5000;
	while(!queue_isEmpty(displayQueue)){
		// wait for the startup sequence to finish
	}
	while(delay){
		delay--;
	}
}
void display_Int(uint16_t value){
	uint16_t digit;
	uint16_t divisor = 10000;
	uint8_t index = 0;
	uint8_t i;
	uint32_t delay = 500000;
	uint8_t temp = 0x80;
	while(divisor != 0){
		/* Get the next digit in the number */
		digit = value / divisor;
		/* Write the digit to local display ram copy */		
		for(i = 0; i < FONT_WIDTH; i++){
			display[HR_Y_START][(FONT_WIDTH * index) + i] = 
											 FONT_BITMAP[digit + NUMBER_OFFSET][i];
		}
		value %= divisor; /* Update the current value */
		divisor /= 10;
		index++;
	}
	enQueue(displayQueue, DISPLAY_WRITE_DATA);
	for(i = DISPLAY_WIDTH; i > 0; i--){
		enQueue(displayQueue, display[HR_Y_START][i]);
	}
	// write 129 bytes to display
	display_Communicate(DISPLAY_WIDTH+1, 0,1);
}

void display_Page(uint8_t display_page){
	int i;
	enQueue(displayQueue, DISPLAY_WRITE_DATA); /* Sending data dump */
	for(i = 0; i < DISPLAY_WIDTH; i++){	/* Enqueue all the data (this row) */
		enQueue(displayQueue, display[display_page][i]);
	}
	display_Communicate(DISPLAY_WIDTH + 1, 0,1); /* Call com functions */
	
}
// Start i2c communication and interrupt after each byte transmition
void display_Communicate(uint16_t numBytes, uint16_t readWrite, uint16_t dataCommand){
	uint32_t interrupt;
	uint32_t tmpReg;

	while(DISPLAY_I2C->ISR & I2C_ISR_BUSY){}	/* wait for current transmission to complete */
	
	interrupt =  DISPLAY_I2C->ISR;
	if(interrupt & I2C_ISR_STOPF){
		DISPLAY_I2C->ICR |= I2C_ICR_STOPCF; //clear flags
	}
	while(DISPLAY_I2C->ISR & I2C_ISR_STOPF){}
		
	tmpReg = DISPLAY_I2C->CR2;
	tmpReg &= (uint32_t)~(I2C_CR2_RESET);
	// enable interrupts and i2c bus
	DISPLAY_I2C->CR1 |= I2C_CR1_TXIE | I2C_CR1_RXIE;
	DISPLAY_I2C->CR1 |= I2C_CR1_PE;
	
	//initiate communication
	tmpReg |= ((I2C_CR2_SADD & DISPLAY_ADDRESS)| I2C_CR2_AUTOEND);
	// set direction and number of bytes
	tmpReg |= ((I2C_CR2_RD_WRN & readWrite) | (I2C_CR2_NBYTES & (numBytes << 16)) | I2C_CR2_START);
	// start i2c once configured
	DISPLAY_I2C->CR2 = tmpReg;
	while(!(DISPLAY_I2C->ISR & I2C_ISR_TXIS)){
			// Wait for TXIS bit
	}
}

void I2C2_EV_IRQHandler(void){
	uint32_t interrupt =  DISPLAY_I2C->ISR;
	if(interrupt & I2C_ISR_STOPF){
		DISPLAY_I2C->ICR |= I2C_ICR_STOPCF; //clear flag
	}
	if(interrupt & I2C_ISR_TC){// transfer complete
		//DISPLAY_I2C->CR2 |= I2C_CR2_STOP; // stop the I2c 
	}else if(interrupt & I2C_ISR_TXE){
		// TX empty, send next byte
		DISPLAY_I2C->TXDR = (uint8_t) deQueue(displayQueue);
	}
	if(interrupt & I2C_ISR_RXNE){
		// RX is not empty, read it before next incoming
	}
}
