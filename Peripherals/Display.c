/*****< display.c >********************************************************/
/*      California Institute of Technology 									  */
/*		EECS53 project 2015   		                          			      */
/*      Portable fitness  tracker                                             */
/*                                                                            */
/*  display.c   -       This display file overcomes the incredible odds       */
/*                      against it by implementing a simple and clean interface*/
/*                      to the rest of the project. A chinese off brand display,*/
/*                      this device is poorly documented and has very unclear */
/*                      initialization instructions. Below is one sequence    */
/*                      that has proven to be repeatedly successful.          */
/*                                                                            */
/*  Author:  Quinn Osha                                                       */
/*                                                                            */
/*** MODIFICATION HISTORY *****************************************************/
/*                                                                            */
/*   mm/dd/yy  F. Lastname    Description of Modification                     */
/*   --------  -----------    ------------------------------------------------*/
/*   03/17/15  Q. Osha        Initial revision.                               */
/*   03/16/15  Q. Osha        Reached turn-on                                 */
/*   03/19/15  Q. Osha        Displayed numbers to screen (sideways)          */
/*   06/05/15  Q. Osha        Displayed numbers and letters(correct dir)      */
/******************************************************************************/

#include "Display.h"
#include "../font.h"
#include <stdint.h>
#include <string.h>
#include "stm32f373xc.h"                  // Device header
#include "../Main/mpu_constants.h"				// MPU general constants
#include "i2c_control.h"	// header for MCU specific i2c constants
#include "../mpu_control.h" // communication method

Queue *displayQueue; 
uint8_t display[DISPLAY_HEIGHT][DISPLAY_WIDTH]; // copy of display RAM
uint8_t current_display_page; // Track where on the display the RAM pointer is

/* Function prototypes */
void display_Page(uint8_t display_page);


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
	
	//EXTI->EMR			 |= (1UL << 23); //dont mask event
	
	
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
	display_Startup();
	
}
void display_Shutdown(){
	GPIOC->ODR		 &= ~((1UL << DISPLAY_RST)  );		/* PC.DISPLAY_RST go low     */
}
void display_Startup(){
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
	
	current_display_page = DISPLAY_PAGEADD_START;
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
}

void display_Int(uint16_t value, uint8_t display_page, uint8_t x_offset, boolean display_now){
	uint16_t digit;
	uint16_t divisor = 10000;
	uint8_t index = 0;
	uint8_t i;
	uint32_t delay = 500000;
	while(divisor != 0){
		/* Get the next digit in the number */
		digit = value / divisor;
		/* Write the digit to local display ram copy */		
		for(i = 0; i < FONT_WIDTH; i++){
			display[display_page][x_offset + (FONT_WIDTH * index) + i] = 
											 FONT_BITMAP[digit + NUMBER_OFFSET][i];
		}
		value %= divisor; /* Update the current value */
		divisor /= 10; /* shift divisor down to get next digit */
		index++;
	}
	if(display_now){ /* Avoid repeatedly writing the whole display. */
		display_Page(display_page); /* Display the page just written to */
	}
}

void display_String(char *string, uint8_t display_page, uint8_t x_offset, boolean display_now){
	uint8_t index, bit;
	uint8_t c;
	for(index = 0; index < strlen(string); index++){
		if(index >= DISPLAY_WIDTH){
			break;
		}
		c = *string++; /* Get the next character in the string */
		/* Write the character to local display ram copy */		
		for(bit = 0; bit < FONT_WIDTH; bit++){
			display[display_page][x_offset + (FONT_WIDTH * index) + bit] = 
											 FONT_BITMAP[c - OFFSET_FROM_ASCII][bit];
		}	
	}
	if(display_now){ /* Avoid repeatedly writing the whole display. */
		display_Page(display_page); /* wait until other pages updated as well */ 
	}
}

void display_Current_Status(){
	
}

void display_Page(uint8_t display_page){
	int i, j, pages_to_read;
	
	//if(display_page >= current_display_page){ /* Addressing mode loops around */
	//	pages_to_read = display_page - current_display_page;
	//}else{
	//	pages_to_read = display_page + (DISPLAY_HEIGHT - current_display_page);
	//}
	
	//for(j = 0; j < pages_to_read + 1; j++){		/* write multiple pages */
		//while(queue_isFull(displayQueue)){ } /* wait for earlier items to be sent */
		enQueue(displayQueue, DISPLAY_WRITE_DATA); /* Sending data dump */
		for(i = DISPLAY_WIDTH-1; i >= 0; i--){	/* Enqueue all the data (this row) */
			//while(queue_isFull(displayQueue)){ } /* wait for earlier items to be sent */
			enQueue(displayQueue, display[current_display_page][i]);
		}
		display_Communicate(DISPLAY_WIDTH + 1, 0,1); /* Call com functions */
		current_display_page++;
		if(current_display_page >= DISPLAY_HEIGHT){
			current_display_page -= DISPLAY_HEIGHT;
		}
	//}
}

// Start i2c communication and interrupt after each byte transmition
void display_Communicate(uint16_t numBytes, uint16_t readWrite, uint16_t dataCommand){
	uint32_t interrupt;
	uint32_t tmpReg;

	while(DISPLAY_I2C->ISR & I2C_ISR_BUSY){}	/* wait for current transmission to complete */
  while(displayQueue->currentSize > numBytes){ } 
	interrupt =  DISPLAY_I2C->ISR;
	while(interrupt & I2C_ISR_TXIS){}
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
	}
	if(interrupt & I2C_ISR_TXE){
		// TX empty, send next byte
		DISPLAY_I2C->TXDR = (uint8_t) deQueue(displayQueue);
	}
	if(interrupt & I2C_ISR_RXNE){
		// RX is not empty, read it before next incoming
	}
}
