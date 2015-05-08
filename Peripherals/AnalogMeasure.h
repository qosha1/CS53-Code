
#include <stdint.h>
#include "Queue.h"


#ifndef ARM_MATH_CM0
#define ARM_MATH_CM0
#endif

#define ANALOG_QUEUE_LENGTH			((uint32_t)0x100)
#define ANALOG_WINDOW_LENGTH		((uint32_t)0x200)
#define MEASUREMENT_TIME  			((uint16_t)15000)// in ms
#define AVERAGE_SAMPLES					1000

typedef struct{
	uint16_t max;
	uint16_t min;
	uint32_t average;
	uint16_t numBeats;
	uint32_t numCounts;
}analogData;

extern Queue *analogQueue;
extern uint32_t isMeasuring;
extern uint32_t isInitialized;


uint32_t Analog_Init(void);
uint32_t Start_Measure(void);
uint32_t Stop_Measure(void);
void update_Beat_Count(void);
analogData * get_Heart_Rate(void);
extern void ADC1_IRQHandler(void);
