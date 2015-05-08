#include <stdint.h>

#define DUTYCYCLE 		1 		//sets duty cycle to 1/(DutyCycle+1)

extern uint16_t analog_Timer_Active;
void init_Analog_Timer(uint16_t measure_Time);
void start_Analog_Timer(void);
void stop_Analog_Timer(void);
extern void TIM15_IRQHandler(void);