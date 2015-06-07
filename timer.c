#include "timer.h"
#include <stdint.h>
#include "stm32f373xc.h"                  // Device header

uint16_t dutyCycleCount = 0;
uint16_t disableTimerFlag = 0;
uint16_t timer_Active = 0;
boolean finished_Flag;


/* This functiono initializes the Timer15 peripheral to count in ms *
 * intervals for ms_counts milliseconds. This function does NOT turn*
 * on the timer. */
void init_Timer(uint16_t ms_counts){
		// intialize the timer for tracking the analog measurement
		// system
	uint16_t prescale  = (SystemCoreClock / 1000); /* prescale to ms */
	uint16_t timerCount = ms_counts;		/* Set the countdown */
	RCC->APB2ENR |= RCC_APB2ENR_TIM15EN; /* turn on timer clock */
	NVIC_SetPriority (TIM15_IRQn, 0x06); /* Set lower interrupt priority */
	TIM15->ARR = timerCount;
	TIM15->PSC = prescale;
	TIM15->SMCR |= TIM_SMCR_TS_1; /* Turn on timer */
}

/* This function starts the Timer15 countdown and enables the 	   *
 * interrupt in the IER. */
void start_Timer(){
	// Enable the counter and turn IER
	NVIC_EnableIRQ(TIM15_IRQn);
	TIM15->DIER |= TIM_DIER_UIE; // turn on interrupts
	TIM15->CR1 |= TIM_CR1_CEN | TIM_CR1_URS; // enable timer
	timer_Active = 1;
	finished_Flag = false;
}

/* This function turns off the Timer15 countdown. It does not disable *
 * the timer. */
void stop_Timer(){
	// tell system to turn off counter after next interrupt
	// to avoid stopping mid-measurement
	disableTimerFlag = 1;
	timer_Active = 0;
}
