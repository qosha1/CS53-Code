
#include <stdint.h>

/* Queue structure definition */
#ifndef QUEUE_H
#define QUEUE_H

typedef struct _Queue
	{
		uint32_t   length; /* Length of queue */
		int16_t *dataArray; /* actual data for queue */
		int16_t headPtr; /* pointer to top of queue */
		int16_t tailPtr; /* pointer to end of queue */ 		
	} Queue;

Queue *initQueue(uint32_t queueLength);
void enQueue(Queue *dataQueue, int16_t data);
int16_t deQueue(Queue *dataQueue);
uint16_t queue_isEmpty(Queue *dataQueue);
uint16_t queue_isFull(Queue *dataQueue);
	
#endif  /* QUEUE_H */
