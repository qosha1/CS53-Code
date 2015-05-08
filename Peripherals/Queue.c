
#include <stdint.h>
#include <stdlib.h>
#include "Queue.h"

/* Queue structure implementation*/

Queue * initQueue(uint32_t queueLength){
	int16_t *data;
	Queue *dataQueue;
	
	dataQueue = (Queue *)malloc(sizeof(Queue));
	data = (int16_t *)malloc(sizeof(int16_t) * queueLength);
	
	if(dataQueue && data){
		dataQueue->dataArray = data;
		dataQueue->headPtr = 0;
		dataQueue->tailPtr = 0; //initialize pointers to queue
		dataQueue->length = queueLength;
	}
	
	return dataQueue; // done and 
}
void enQueue(Queue *dataQueue, int16_t data){
	if(!queue_isFull(dataQueue)){
		dataQueue->dataArray[dataQueue->tailPtr] = data;

		if(dataQueue->tailPtr == dataQueue->length - 1){
			dataQueue->tailPtr = 0;// reset to beginning of array
		}else{
			dataQueue->tailPtr++;// otherwise just increment pointer
		}
	}
}
int16_t deQueue(Queue *dataQueue){
	int16_t data;
	if(!queue_isEmpty(dataQueue)){
		data = dataQueue->dataArray[dataQueue->headPtr];

		if(dataQueue->headPtr == dataQueue->length - 1){
			dataQueue->headPtr = 0;// reset to beginning of array
		}else{
			dataQueue->headPtr++;// otherwise just increment pointer
		}
		return data;
	}else{
		return 0;
	}
	
	
}
uint16_t queue_isEmpty(Queue *dataQueue){
	// if tail = head, queue is empty
	// postive -> is empty
	// zero -> not empty
	if(dataQueue->headPtr ^ dataQueue->tailPtr){
		return 0;
	}else{
		return 1;
	}
}
// Return non-zero if the queue is full, 0 if its empty
uint16_t queue_isFull(Queue *dataQueue){
		if(dataQueue->tailPtr == dataQueue->length - 1){
			// zero if head pointer isnt at first slot
			// postive otherwise (its full)
			if(dataQueue->headPtr == 0 ){
				return 1;
			}else{
				return 0;
			}
		}else{
			if((dataQueue->tailPtr + 1)^ dataQueue->headPtr){
				return 0;
			}else{
				return 1;
			}
		}
		
}
