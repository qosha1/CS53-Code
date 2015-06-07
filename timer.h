#ifndef _timer_
#define _timer_

#include <stdint.h>
#include "../Main/mpu_constants.h"

#define DUTYCYCLE 		1 		//sets duty cycle to 1/(DutyCycle+1)
#define TIMER_PRESCALE		((uint16_t) 16000)

extern uint16_t timer_Active;
extern boolean finished_Flag;

void init_Timer(uint16_t measure_Time);
void start_Timer(void);
void stop_Timer(void);

#endif //_timer_