/*----------------------------------------------------------------------------
 * 'main' function template
 *---------------------------------------------------------------------------*/

#include <stdlib.h>
#include "stm32f3xx.h"                // Device header
#include "../Peripherals/Cap_buttons.h"								// user input control
#include "mpu_constants.h"							//mpu specific constants 
#include "../Peripherals/Display.h"			//display module
#include "../mpu_control.h"							// Control functions for MPU
#include "../Peripherals/i2c_control.h" // MPU communication methods
//#include "../Peripherals/Queue.h"
static enum keyCode getKey(void); //Local function dec

enum state_type {
	STATUS_IDLE,
	STATUS_MEASURE,
	STATUS_INVALID
};


void SystemCoreClockSetHSI(void) {
	// initialize system clock
	RCC->CR |= ((uint32_t)RCC_CR_HSION);                      /* Enable HSI                        */
  while ((RCC->CR & RCC_CR_HSIRDY) == 0);                   /* Wait for HSI Ready                */

  RCC->CFGR = RCC_CFGR_SW_HSI;                              /* HSI is system clock               */
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);   /* Wait for HSI used as system clock */
	RCC->CR &= ~RCC_CR_HSEON;
	
  RCC->CFGR |= RCC_CFGR_HPRE_DIV1;                          /* HCLK = SYSCLK                     */

  RCC->CR &= ~RCC_CR_PLLON;                                 /* Disable PLL */

  /*  PLL configuration:  = HSI/2 * 4 = 16 MHz */
  RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMUL);
  RCC->CFGR |=  RCC_CFGR_PLLMUL4;

  RCC->CR |= RCC_CR_PLLON;                                  /* Enable PLL                        */
  while((RCC->CR & RCC_CR_PLLRDY) == 0) __NOP();            /* Wait till PLL is ready            */

  RCC->CFGR &= ~RCC_CFGR_SW;                                /* Select PLL as system clock source */
  RCC->CFGR |=  RCC_CFGR_SW_PLL;

  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);   /* Wait till PLL is system clock src */
	//RCC->CFGR3 |= RCC_CFGR3_I2C2SW | RCC_CFGR3_I2C1SW; 				/* Set the I2c clocks to sysclk */
}


/*
 * main: initialize and start the system
 * Flash memory : 128 KB
 * SRAM 			  :  24 Kb
 */
int main (void) {
	static const uint32_t states[] = 
	{
			STATUS_IDLE,
			STATUS_MEASURE,
			STATUS_INVALID,
			STATUS_INVALID,
			STATUS_INVALID
	};	
	enum keyCode key;
  enum state_type curState =  STATUS_MEASURE;
	enum state_type prevState = STATUS_IDLE; // default to idle
	
	SystemCoreClockSetHSI();								/* Setup the global clock */
  SystemCoreClockUpdate();                /* Get Core Clock Frequency   */
	if(SysTick_Config(SystemCoreClock / 1000)){	/* Set the systick to ms */
		while(1);
	}
	NVIC_EnableIRQ(I2C2_EV_IRQn); //Display interrupt
	NVIC_EnableIRQ(I2C1_EV_IRQn); //MPU motion control interrupt
	NVIC_EnableIRQ(USART1_IRQn);	//BT USART interrupt
	
	//Button_Initialize();    // init the capacitive buttons
	display_Init();		// turn on display
	mpu_I2C_Init();		// initialize i2c peripheral
	mpu_init();		// Initialize the mpu registers
	//display_Int(15);	// Sample display data
	
	//while(queue_isEmpty(mpuRxQueue)){ __NOP(); }
	//display_Int(deQueue(mpuRxQueue));
	
  while(1) {// simple FSM
		//if(Buttons_KeyAvailable()){
		//	get_Key();
		//	key = getKey();
			
		//	curState = states[key];
		//}
		
		//if(curState != prevState){ //update functions
			if(curState == STATUS_IDLE){
				//do nothing currently
				//possible to enter low power mode
			}
	}
	
}


static enum keyCode getKey(void)
{
	static const enum keyCode keycodes[] = {
		KEYCODE_ON,
		KEYCODE_MEASURE,
		KEYCODE_EXTRA1,
		KEYCODE_EXTRA2,
		KEYCODE_ILLEGAL
	};
	
	int key;
	
	key = Buttons_GetKey(); // get key value
	key = key >> 1;
	if(key & 0x08){
		//key 4
		key = 3;
	}else if(key & 0x04){
		//key 3
		key = 2;
	}else if(key & 0x02){
		//key 2
		key = 1;
	}else if(key & 0x01){
		//key 1
		key = 0;
	}else {
		//key invalid
		key = 4;
	}
	return keycodes[key]; // return keycode
}
