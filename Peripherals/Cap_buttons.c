
#include <stdint.h>
#include "stm32f373xc.h"                  // Device header
#include "Cap_Buttons.h"
#include "Display.h"

enum button_type keyPress;
Queue *buttonQueue;
volatile uint8_t keyStatus;
volatile uint8_t buttonAvailable = 0;

int32_t Button_Initialize (void) {
	uint32_t delay = 5000000;
	uint32_t tmpReg;
	buttonQueue = initQueue(BUTTON_QUEUE_LENGTH); 
  RCC->AHBENR  	 |= RCC_AHBENR_GPIOAEN;  /* Enable GPIOA clock  */
	RCC->AHBENR  	 |= RCC_AHBENR_GPIOFEN;  /* Enable GPIOF clock  */
	
  RCC->AHBENR  	 |= RCC_AHBENR_GPIOBEN;  /* Enable GPIOB clock  */
	RCC->APB1ENR   |= RCC_APB1ENR_I2C1EN;  /* Enable I2C_1 clock*/

	// Interrupt signal
	GPIOA->MODER    &=  ~((3UL << 2*15)  );         /* PA.15 is input              */
	GPIOA->PUPDR    &=  ~((3UL << 2*15)  );         /* PA.15 is floating         */
	
	GPIOB->AFR[0]  |= (1UL << 4*6);		// alternate function 1
	GPIOB->AFR[0]  |= (1UL << 4*7);		// alternate function 1
	
	GPIOB->OTYPER |= (1UL << 1*6);		//  open drain
	GPIOB->OTYPER |= (1UL << 1*7);		//  open drain
	
	GPIOB->OSPEEDR |= (3UL << 2*6);		//  high speed
	GPIOB->OSPEEDR |= (3UL << 2*7);		//  high speed
	
	GPIOB->MODER |= (2UL << 2*6)  ;         /* Pb.6 is i2c1 clk*/
	GPIOB->MODER |= (2UL << 2*7)  ;         /* Pb.7 is i2c1 data*/
	
  GPIOF->MODER    &= ~((3UL << 2*0)  ) ;         /* PF.0 is input             */
  GPIOF->PUPDR    &=  ~(3UL << 2*0);         /* PF.0 is floating     */
  GPIOF->MODER    &= ~((3UL << 2*1)  ) ;         /* PF.1 is input             */
  GPIOF->PUPDR    &=  ~(3UL << 2*1);         /* PF.1 is floating        */
	
	//GPIOF->PUPDR = '00' // outputs dont have pullup or pulldown

	RCC->APB1RSTR   |= RCC_APB1RSTR_I2C1RST;  /* reset I2C_1 clock*/
	RCC->APB1RSTR   &= ~RCC_APB1RSTR_I2C1RST;  /* release reset I2C_1	clock*/
	
	while(!(RCC->APB1ENR & RCC_APB1ENR_I2C1EN)){
	}
	// Set i2c SCL frequency 
	I2C1->TIMINGR  |= (I2C_TIMING_PRESC << 28); // presecale pin clock 
	I2C1->TIMINGR  |= (I2C_TIMING_SCLH << 8) | (I2C_TIMING_SCLL << 0);
	I2C1->TIMINGR  |= (I2C_TIMING_SCLDEL << 20) | (I2C_TIMING_SDADEL << 16);   

	enQueue(buttonQueue, BUTTON_RESET_REG); // reset the buttons
	enQueue(buttonQueue, 1);// non-zero value reset 
	//initiate communication
	I2C1->CR2 |= (I2C_CR2_SADD & BUTTON_ADDRESS)| I2C_CR2_AUTOEND;
	// set direction and number of bytes
	I2C1->CR2 |= (I2C_CR2_RD_WRN & BUTTON_WRITE) | (I2C_CR2_NBYTES & (2UL <<16));
	I2C1->CR1 |= I2C_CR1_TXIE | I2C_CR1_RXIE | I2C_CR1_TCIE;
	I2C1->CR1 |= I2C_CR1_PE;
	I2C1->CR2 |= I2C_CR2_START; // start i2c once configured
	while(!queue_isEmpty(buttonQueue)){
	}
	I2C1->ICR |= I2C_ICR_STOPCF;
	while(I2C1->ISR & I2C_ISR_STOPF){
	}
	while(delay){
		delay--;
	}
	
	// setup the touch sensor
	enQueue(buttonQueue, BUTTON_THRESHOLD_REG0);
	enQueue(buttonQueue, BUTTON_THRESHOLD);
	enQueue(buttonQueue, BUTTON_THRESHOLD);
	enQueue(buttonQueue, BUTTON_THRESHOLD);
	enQueue(buttonQueue, BUTTON_THRESHOLD);
	enQueue(buttonQueue, BUTTON_THRESHOLD);
	
	tmpReg = I2C1->CR2;
	tmpReg &= (0xF8UL << 24);
	// Handle button interrupts
	enQueue(buttonQueue, BUTTON_KEY_REG);
	//initiate communication
	tmpReg |= ((I2C_CR2_SADD & BUTTON_ADDRESS)| I2C_CR2_AUTOEND);
	// set direction and number of bytes
	tmpReg |= ((I2C_CR2_RD_WRN & BUTTON_WRITE) | (I2C_CR2_NBYTES & (6UL << 16)) | I2C_CR2_START);
	// start i2c once configured
	I2C1->CR2 = tmpReg;
	while(!queue_isEmpty(buttonQueue)){
	}
	delay = 5000000;
	while(delay){
		delay--;
	}	
		
	//RCC->APB2ENR	 		 |= RCC_APB2ENR_SYSCFGCOMPEN;	
	//while(!(RCC->APB2ENR & RCC_APB2ENR_SYSCFGCOMPEN)){}
	//SYSCFG->EXTICR[3]  |=  (0UL << 12); //Set PA.15 as external interrupt 15
	//EXTI->IMR					 |= (1UL << 15); //dont mask interrupt
	//EXTI->IMR					 |= (1UL << 15); // dont mask events
	//EXTI->FTSR				 |= (1UL << 15);// watch for falling edges

	if(buttonAvailable || !(GPIOA->IDR & (1UL << 1*15))){
			get_Key();
	}
		
		
  return (1);
}
//
int32_t Buttons_GetKey (void){
	 return keyStatus;
 }
void get_Key(void){
	if(buttonAvailable){
		uint32_t delay = 100000;
		uint32_t tmpReg;
		I2C1->CR1 &= ~I2C_CR1_PE;
		while(I2C1->CR1 & I2C_CR1_PE){
		}
		// Set i2c SCL frequency 
		I2C1->TIMINGR  |= (I2C_TIMING_PRESC << 28); // presecale pin clock 
		I2C1->TIMINGR  |= (I2C_TIMING_SCLH << 8) | (I2C_TIMING_SCLL << 0);
		I2C1->TIMINGR  |= (I2C_TIMING_SCLDEL << 20) | (I2C_TIMING_SDADEL << 16);   
		I2C1->CR1 |= I2C_CR1_TXIE | I2C_CR1_RXIE | I2C_CR1_TCIE;
		I2C1->CR1 |= I2C_CR1_PE;
		while(!(I2C1->CR1 & I2C_CR1_PE)){
		}
		while(delay){
			delay--;
		}
		tmpReg = I2C1->CR2;
		tmpReg &= (0xF8UL << 24);
		// Handle button interrupts
		enQueue(buttonQueue, BUTTON_KEY_REG);
		//initiate communication
		tmpReg |= ((I2C_CR2_SADD & BUTTON_ADDRESS)| I2C_CR2_AUTOEND);
		// set direction and number of bytes
		tmpReg |= ((I2C_CR2_RD_WRN & BUTTON_WRITE) | (I2C_CR2_NBYTES & (1UL << 16)) | I2C_CR2_START);
		// start i2c once configured
		I2C1->CR2 = tmpReg;
		while(!(I2C1->ISR & I2C_ISR_TXIS) && !queue_isEmpty(buttonQueue)){
				// Wait for TXIS bit
				//I2C1->TXDR = (uint8_t) deQueue(buttonQueue);
		}
		delay = 100000;
		while(delay){
			delay--;
		}
		tmpReg = I2C1->CR2;
		tmpReg &= (0xF8UL << 24);		
		tmpReg |= (I2C_CR2_SADD & (BUTTON_ADDRESS | (1UL)))| I2C_CR2_AUTOEND;
		// set direction and number of bytes
		tmpReg |= ((I2C_CR2_RD_WRN) | (I2C_CR2_NBYTES & (2UL << 16)) | I2C_CR2_START);
		// start i2c once configured
		I2C1->CR2 = tmpReg;
		buttonAvailable = 0;
		while(!(GPIOA->IDR & (1UL << 1*15))){
			//wait for removal of interrupt
		}
		I2C1->CR1 &= ~I2C_CR1_PE;
	}
}
int32_t Buttons_KeyAvailable(void){
	if(buttonAvailable || !(GPIOA->IDR & (1UL << 1*15))){
		buttonAvailable = 1;
	}
	return buttonAvailable;
}
/*void EXTI4_15_IRQHandler(void){
	buttonAvailable = 1;
}*/

