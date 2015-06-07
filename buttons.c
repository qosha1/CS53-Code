/*****< buttons.c >********************************************************/
/*      California Institute of Technology 									  */
/*		EECS53 project 2015   		                          			      */
/*      Portable fitness  tracker                                             */
/*                                                                            */
/*  buttons.c   -       This file implements the capacitive touch buttons     */
/*                      using the touch sensing controller on board the STM32.*/
/*                                                                            */
/*  Author:  Quinn Osha                                                       */
/*                                                                            */
/*** MODIFICATION HISTORY *****************************************************/
/*                                                                            */
/*   mm/dd/yy  F. Lastname    Description of Modification                     */
/*   --------  -----------    ------------------------------------------------*/
/*   05/25/15  Q. Osha        Initial revision.                               */
/*   04/01/15  Q. Osha        Add basic implemenation and fix errors          */
/*   06/05/15  Q. Osha        Clean up code and alter sensor settings         */
/******************************************************************************/

#include <stdlib.h>
#include "buttons.h"
#include "stm32f373xc.h"            // Device header
#include "../Main/mpu_constants.h"
#include "../timer.h"               // Timer init and control functions

/* All the buttons in the analog sensing group */
const struct _Button_Group buttons = 
{
	.button1 = CAP_BUTTON1,
	.button2 = CAP_BUTTON2,
	.button3 = CAP_BUTTON3,
	.button4 = CAP_BUTTON4,
};
/* The button that is to be used as the measuring (comparator) capacitor */
const uint8_t channel_button[4] = 
{
	CAP_BUTTON2,
	CAP_BUTTON1,
	CAP_BUTTON4,
	CAP_BUTTON3
};
uint32_t no_press_threshold; /* Count threshold for determining a press */
uint8_t last_button; /* save the last output of get_Button() for dbouncing */

/* Prototypes */ 
void set_Sampling_Capacitor(uint8_t button);
void set_Measure_Channel(uint8_t button);
uint8_t get_Single_Button(uint8_t button);
void set_Threshold_Value();

void buttons_Init(){
	
	RCC->AHBENR	  |= RCC_AHBENR_GPIOAEN; 			/* Enable GPIOA clock*/
	RCC->AHBENR   |= RCC_AHBENR_TSCEN;  	/* Enable TSC clock*/

	GPIOA->AFR[0]  |= (((uint32_t) CAP_BUTTON_AF) << 4 * (buttons.button1 % 8));// alternate function setup
	GPIOA->AFR[0]  |= (((uint32_t) CAP_BUTTON_AF) << 4 * (buttons.button2 % 8));// alternate function setup
	GPIOA->AFR[0]  |= (((uint32_t) CAP_BUTTON_AF) << 4 * (buttons.button3 % 8));// alternate function setup
	GPIOA->AFR[0]  |= (((uint32_t) CAP_BUTTON_AF) << 4 * (buttons.button4 % 8));// alternate function setup
	
	GPIOA->MODER	 |= (2UL << 2*buttons.button1)  ;         /* SET TO ALTERNATE FUNCTION	*/
	GPIOA->MODER 	 |= (2UL << 2*buttons.button2)  ;         /* SET TO ALTERNATE FUNCTION */
	GPIOA->MODER	 |= (2UL << 2*buttons.button3)  ;         /* SET TO ALTERNATE FUNCTION	*/
	GPIOA->MODER 	 |= (2UL << 2*buttons.button4)  ;         /* SET TO ALTERNATE FUNCTION */
		
	TSC->IER 		|= TSC_IER_EOAIE; /* interrupt enables */
	TSC->IOGCSR |= TSC_IOGCSR_G1E; /* Enable group 1 */
	TSC->CR			|= 	TSC_CR_MCV_2 | TSC_CR_PGPSC_2 
								| TSC_CR_CTPH_0 |TSC_CR_CTPL_0; /* Set max error count */
	TSC->CR  		|= TSC_CR_TSCE; /* Enable to controller */
	
	init_Timer(KEY_DEBOUNCE_MS);
	last_button = 0;
	set_Threshold_Value();
}
/* This function returns an 8 bit value that contains any registered *
 * presses at the time. The return value encodes each bit with the   *
 * result of that button (eg. bit2 represents button2)							 */
uint8_t get_Button(){
	/* Acquisition sequence 
	 * 1. Set one sampling capacitor and close other analog switches  *
	 * 		in the group. Set as open drain (sampling)low to discharge 	*
	 *		caps.																												*
	 * 2. Leave inputs floating																				*
	 * 3. Output (1) electrode push pull high to charge that electrode*
	 * 4. Floating																										*
	 * 5. Input floating with analog switch closed for sampling and   *
	 *		measured electrodes.																				*
	 * 6. Floating																										*
	 * 7. Floating, measure C_s voltage     													*/

		uint8_t button_presses = 0; /* Varaible to store any presses */
		button_presses = get_Single_Button(buttons.button1);
		button_presses |= get_Single_Button(buttons.button2);
		button_presses |= get_Single_Button(buttons.button3);
		button_presses |= get_Single_Button(buttons.button4);
		if((button_presses != last_button) && button_presses){
			last_button = button_presses;
			
			//start_Timer();
			return button_presses;
		}
		return NO_BUTTON;
}

uint8_t get_Single_Button(uint8_t button){
	uint32_t sample_count = 0;
	
	set_Sampling_Capacitor(button);
	set_Measure_Channel(channel_button[button]);
	TSC->ICR |= (TSC_ISR_EOAF | TSC_ISR_MCEF); /* Clear status register */
	TSC->CR |= TSC_CR_START;				  /* Start the acquisition sequence */ 
	
	while(!TSC->ISR){} /* Wait until finished */
	if(!(TSC->ISR & TSC_ISR_MCEF)){ /* Didn't have an error */
		TSC->ICR |= (TSC_ISR_EOAF | TSC_ISR_MCEF); /* Clear Status */
		sample_count = TSC->IOGXCR[0];
		if(sample_count > no_press_threshold){
			return 1 << button; /* Bitwise return value */
		}
		
	}else{ /* Had an error. Clear status reg and finish */
		TSC->ICR |= (TSC_ISR_EOAF | TSC_ISR_MCEF);
	}
	
	return 0;
	
}

/* This function initializes the no_press_threshold value for *
 * comparison with future values. It assumes that	no buttons 	*
 * are being pressed at the time. 														*/
void set_Threshold_Value(){
	uint32_t sample_count = 0;
	uint8_t	 successful_measures = 0;
	uint8_t i;
	for(i = 0; i < sizeof(buttons); i++){
		set_Sampling_Capacitor(i);
		set_Measure_Channel(channel_button[i]);
		TSC->ICR |= (TSC_ISR_EOAF | TSC_ISR_MCEF); /* Clear status register */
		TSC->CR |= TSC_CR_START;				  /* Start the acquisition sequence */ 
		while(!TSC->ISR){} /* Wait until finished */
		if(!(TSC->ISR & TSC_ISR_MCEF)){ /* Didn't have an error */
			TSC->ICR |= (TSC_ISR_EOAF | TSC_ISR_MCEF); /* Clear Status */
			sample_count += TSC->IOGXCR[0];
			successful_measures++;
		}else{ /* Had an error. Clear status reg and finish */
			TSC->ICR |= (TSC_ISR_EOAF | TSC_ISR_MCEF);
		}
		
	}
	no_press_threshold = sample_count / successful_measures; /* Take the average */
}
void set_Sampling_Capacitor(uint8_t button){
	uint32_t TSC_IOSCR_ALL = TSC_IOSCR_G1_IO1 | TSC_IOSCR_G1_IO2 
												 | TSC_IOSCR_G1_IO3 | TSC_IOSCR_G1_IO4;
	
	TSC->IOSCR &= ~TSC_IOSCR_ALL; 	

	switch(button) /* Only set for one button */
	{
	case CAP_BUTTON1:
		GPIOA->OTYPER  |= (1UL << 1*buttons.button1); /* set as open drain */
		TSC->IOSCR 		 |= TSC_IOSCR_G1_IO1; /* set sampling capacitors */
		break;
	case CAP_BUTTON2:
		GPIOA->OTYPER  |= (1UL << 1*buttons.button2); /* set as open drain */
		TSC->IOSCR 		 |= TSC_IOSCR_G1_IO2; /* set sampling capacitors */
		break;
	case CAP_BUTTON3:
		GPIOA->OTYPER  |= (1UL << 1*buttons.button3); /* set as open drain */
		TSC->IOSCR 		 |= TSC_IOSCR_G1_IO3; /* set sampling capacitors */
		break;
	case CAP_BUTTON4:
		GPIOA->OTYPER  |= (1UL << 1*buttons.button4); /* set as open drain */
		TSC->IOSCR 		 |= TSC_IOSCR_G1_IO4; /* set sampling capacitors */
	}
	
}
void set_Measure_Channel(uint8_t button){
	uint32_t TSC_IOCCR_ALL = TSC_IOCCR_G1_IO1 | TSC_IOCCR_G1_IO2 
												 | TSC_IOCCR_G1_IO3 | TSC_IOCCR_G1_IO4;
	
	TSC->IOCCR &= ~TSC_IOCCR_ALL; 
	switch(button) /* Only set for one button */
	{
	case CAP_BUTTON1:
		GPIOA->OTYPER  &= ~(3UL << 1*buttons.button1); /* set as pushpull */
		TSC->IOCCR |= TSC_IOCCR_G1_IO1;	/* Set as TSC channel */
		break;
	case CAP_BUTTON2:
		GPIOA->OTYPER  &= ~(3UL << 1*buttons.button2); /* set as pushpull */
		TSC->IOCCR |= TSC_IOCCR_G1_IO2;	/* Set as TSC channel */
		break;
	case CAP_BUTTON3:
		GPIOA->OTYPER  &= ~(3UL << 1*buttons.button3); /* set as pushpull */
		TSC->IOCCR |= TSC_IOCCR_G1_IO3;	/* Set as TSC channel */
		break;
	case CAP_BUTTON4:
		GPIOA->OTYPER  &= ~(3UL << 1*buttons.button4); /* set as pushpull */
		TSC->IOCCR |= TSC_IOCCR_G1_IO4;	/* Set as TSC channel */
	}
}