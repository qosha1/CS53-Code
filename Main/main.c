/*****< main.c >***************************************************************/
/*      California Institute of Technology 									  */
/*		EECS53 project 2015   		                          			      */
/*      Portable fitness  tracker                                             */
/*                                                                            */
/*  Main.c -        This file is the main() call made by the low level init   */
/*                  code. The user inteface is mostly contained within this   */
/*                  file. It implements a very simple FSM that moves to states*/
/*                  based on the current state and the user input using the   */
/*                  buttons. */
/*                                                                            */
/*  Author:  Quinn Osha                                                       */
/*                                                                            */
/*** MODIFICATION HISTORY *****************************************************/
/*                                                                            */
/*   mm/dd/yy  F. Lastname    Description of Modification                     */
/*   --------  -----------    ------------------------------------------------*/
/*   03/15/15  Q. Osha        Initial revision.                               */
/*   04/10/15  Q. Osha        Add display init code                           */
/*   05/05/15  Q. Osha        Add motion control init code                    */
/*   05/08/15  Q. Osha        Add bluetooth control init code                 */
/*   06/04/15  Q. Osha        Remove bluetooth code due to hardware issues    */
/******************************************************************************/

#include <stdlib.h>
#include "stm32f3xx.h"                // Device header
#include "mpu_constants.h"							//mpu specific constants 
#include "../Peripherals/Display.h"			//display module
#include "../mpu_control.h"							// Control functions for MPU
#include "../Peripherals/i2c_control.h" // MPU communication methods
//#include "../bt_control.h" 							// bluetooth Functionality
#include "../buttons.h"
#include "../timer.h"										// Timer init and control functions

//#include "../Peripherals/Queue.h"
static enum keyCode getKey(void); //Local function dec
static enum state_type getState(enum state_type curState, enum keyCode key);

enum state_type {
	STATUS_OFF,
	STATUS_IDLE,
	STATUS_MEASURE,
	STATUS_INVALID
};
enum keyCode {
	KEYCODE_ON,
	KEYCODE_MEASURE,
	KEYCODE_EXTRA1,
	KEYCODE_EXTRA2,
	KEYCODE_NOPRESS
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
	int button_press;
	enum keyCode key;
  enum state_type curState =  STATUS_MEASURE;
	enum state_type prevState = STATUS_IDLE; // default to idle
	mpu_data_s *mpu_data = malloc(sizeof(mpu_data_s));
	
	
	SystemCoreClockSetHSI();								/* Setup the global clock */
  SystemCoreClockUpdate();                /* Get Core Clock Frequency   */
	if(SysTick_Config(SystemCoreClock / 1000)){	/* Set the systick to ms */
		while(1);
	}
	NVIC_EnableIRQ(I2C2_EV_IRQn); //Display interrupt
	NVIC_EnableIRQ(I2C1_EV_IRQn); //MPU motion control interrupt
	NVIC_EnableIRQ(USART1_IRQn);	//BT USART interrupt

	//bt_Init();
	buttons_Init();	/* Initialize button GPIOs and functions */
	display_Init();		// turn on display

	display_Int(15, 0, 0, true);
	display_Int(20, 1, 0, true);
	display_Int(25, 2, 0, true);
	display_Int(175, 3, 0, true);
	display_Int(101, 4, 0 , true );
	Delay(20);
	display_Int(1592, 5, 0 , true);
	Delay(20);
	display_Int(12, 0, 0, true);
	
	//mpu_I2C_Init();		// initialize i2c peripheral
	//mpu_init();		// Initialize the mpu registers
	
  while(1) {// simple FSM
		key = getKey(); /* Get latest key code */
		if(key != KEYCODE_NOPRESS){ /* Check if there was a valid press */
			curState = getState(curState, key); /* Yup, update state based on input */
			
			if(curState == STATUS_IDLE){ 						/************* IDLE ***********/
				if(curState != prevState){ 
						if(prevState == STATUS_MEASURE){
							stop_Measuring();	//turn off acquisition functions
							display_String("lMeas", 0, 0, true);
						}
				display_String("Idle", 0, 0, true);
				}
			}else if(curState == STATUS_MEASURE){		/************* MEASURE ***********/
				//turn on measurement data. 
				if(curState != prevState){ //turn on acquisition functions
					start_Measuring();
					display_String("nowMeas", 1, 0, true);
				}else{ // already measuring
					/* Display data in cool way */
					display_String("stlMeas", 0, 0, true);
					if(data_Packet_Ready()){
						get_Data_Packet(mpu_data, NEWEST_DATA_PACKET);
					}
					if(finished_Flag){
						display_Motion_Data(mpu_data);
						//display_Int(mpu_data->gyro_x);
						//display_Int(mpu_data->accel_x);
						stop_Timer(); /* Reset timer */
						start_Timer();
					}
				}
			}else if(curState == STATUS_OFF){					/************* OFF ***********/
				//display_Shutdown(); /* Turn of the main display */
				//stop_Measuring();
				display_String("OFF", 0, 0, true);
			}
			prevState = curState;
		}
	}
}

static enum state_type getState(enum state_type curState, enum keyCode key){
	
	static const enum state_type 
			state_Transition[NUM_CAP_BUTTONS][NUM_CAP_BUTTONS] = {
					{STATUS_IDLE,     STATUS_OFF,    STATUS_OFF, 			 STATUS_OFF},	/* STATUS_OFF */
					{ STATUS_OFF, STATUS_MEASURE,    STATUS_IDLE, 		STATUS_IDLE},	/* STATUS_IDLE */
					{ STATUS_OFF,    STATUS_IDLE, STATUS_MEASURE,  STATUS_MEASURE}, /* STATUS_MEASURE */
					{ STATUS_OFF,    STATUS_IDLE,    STATUS_IDLE, 	  STATUS_IDLE}	/* STATUS_INVALID */
					};
			
	return state_Transition[curState][key];
}

static enum keyCode getKey(void)
{
	static const enum keyCode keycodes[5] = {
		KEYCODE_ON,
		KEYCODE_MEASURE,
		KEYCODE_EXTRA1,
		KEYCODE_EXTRA2,
		KEYCODE_NOPRESS};
	
	uint8_t key;
	
	key = get_Button(); // get button presses
	if(key & CAP_BUTTON1_MASK){
		key = CAP_BUTTON1;
	}else if(key & CAP_BUTTON2_MASK){
		key = CAP_BUTTON2;
	}else if(key & CAP_BUTTON3_MASK){
		key = CAP_BUTTON3;
	}else if(key & CAP_BUTTON4_MASK){
		key = CAP_BUTTON4;
	}else {
		//key idle
		key = 4;
	}
	return keycodes[key]; // return keycode
}


/* This function resets the core peripherals upon reset of the cpu. */
void resetState(void){
	NVIC_DisableIRQ(EXTI9_5_IRQn);/* we don't want to interrupt during setup */	
	RCC->APB2ENR	 		 |= RCC_APB2ENR_SYSCFGEN;				/* Turn on sysconfig clock */
	
	while(!(RCC->APB2ENR & RCC_APB2ENR_SYSCFGEN)){}
	SYSCFG->EXTICR[1]  &= ~(1UL << 4); 				//Set MPU_INTRPT as external interrupt 5
	EXTI->IMR					 &= ~(1UL << MPU_INTRPT);//dont mask interrupt
	EXTI->FTSR				 &= ~(1UL << MPU_INTRPT);// watch for falling edges
	
		
	RCC->APB1RSTR 		 |= RCC_APB1RSTR_I2C1RST 
											| RCC_APB1RSTR_I2C2RST;	
	RCC->APB2RSTR 		 |= RCC_APB2RSTR_SYSCFGRST 
											| RCC_APB2RSTR_USART1RST 
											| RCC_APB2RSTR_TIM15RST;
	RCC->APB1RSTR 		 &= ~(RCC_APB1RSTR_I2C1RST 
											| RCC_APB1RSTR_I2C2RST);	
	RCC->APB2RSTR 		 &= ~(RCC_APB2RSTR_SYSCFGRST 
											| RCC_APB2RSTR_USART1RST 
											| RCC_APB2RSTR_TIM15RST);

}
