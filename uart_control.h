#include "../Peripherals/Queue.h"
#include "stm32f373xc.h" // Device header

void init_Uart(USART_TypeDef *USART, Queue *Usart_RxQueue, Queue *Usart_TxQueue, 
								uint32_t divider);
void send_Data(USART_TypeDef *USART);

/* Interrupt handlers for USART devices */
extern void USART1_IRQHandler(void);
extern void USART2_IRQHandler(void);