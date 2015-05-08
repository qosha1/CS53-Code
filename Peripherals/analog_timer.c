#include "analog_timer.h"
#include <stdint.h>
#include "stm32f373xc.h"                  // Device header
#include "AnalogMeasure.h"

uint16_t dutyCycleCount = 0;
uint16_t disableTimerFlag = 0;
uint16_t analog_Timer_Active = 0;
uint16_t finishedFlag = 0;

void init_Analog_Timer(uint16_t measure_Time){
		// intialize the timer for tracking the analog measurement
		// system
	uint16_t prescale  = (uint16_t)(8000); // prescale down to 1Khz
	uint16_t timerCount = measure_Time; //with 1 Khz clk, time in ms is count
	RCC->APB2ENR |= RCC_APB2ENR_TIM15EN; // turn on timer clock
	
	TIM15->ARR = timerCount;
	TIM15->PSC = prescale;
	TIM15->SMCR |= TIM_SMCR_TS_1;
}

void start_Analog_Timer(){
	// Enable the counter and turn IER
	TIM15->DIER |= TIM_DIER_UIE; // turn on interrupts
	TIM15->CR1 |= TIM_CR1_CEN | TIM_CR1_URS; // enable timer
	analog_Timer_Active = 1;
}

void stop_Analog_Timer(){
	// tell system to turn off counter after next interrupt
	// to avoid stopping mid-measurement
	disableTimerFlag = 1;
	analog_Timer_Active = 0;
}
void TIM15_IRQHandler(void) {
		//handle interrupts
		if(TIM15->SR & TIM_SR_UIF) {
			// handle the update interrupt
			if(isMeasuring){
				Stop_Measure();// turn off analog measuring
				dutyCycleCount = 0; // reset duty cycle counter
				finishedFlag = 1;
			}else{
				//start measuring or wait depending on duty cycle
				if(dutyCycleCount == DUTYCYCLE){
					Start_Measure();
				}else{
					dutyCycleCount++;
				}
			}
			if(disableTimerFlag){
					TIM15->DIER &= ~TIM_DIER_UIE;// turn off interrupts
					TIM15->CR1 &= ~TIM_CR1_CEN;// disable timer
					Stop_Measure(); // turn off the analog circuits
					disableTimerFlag = 0;
			}
		}
		
		TIM15->SR &=  ~(TIM_SR_UIF |TIM_SR_CC2IF | TIM_SR_CC1IF); // reset status register
}
