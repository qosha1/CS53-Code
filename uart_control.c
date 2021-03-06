/*****< uart_control.c >*******************************************************/
/*      California Institute of Technology 									  */
/*		EECS53 project 2015   		                          			      */
/*      Portable fitness  tracker                                             */
/*                                                                            */
/*  uart_control.c - This file contains functions that allow for simple       */
/*                      simple implementation of the ARM USART peripherals.   */
/*                      using the typical configuration with the ability to   */
/*                      update the baud rate and other parameters using separate
/*                      functions allows for quick implementations.           */
/*                                                                            */
/*  Author:  Quinn Osha                                                       */
/*                                                                            */
/*** MODIFICATION HISTORY *****************************************************/
/*                                                                            */
/*   mm/dd/yy  F. Lastname    Description of Modification                     */
/*   --------  -----------    ------------------------------------------------*/
/*   date_edi  Q. Osha        Initial revision.                               */
/*   date_edi  Q. Osha        edits_made_to_file                              */
/******************************************************************************/


#include "uart_control.h" // Uart specific header file
#include "stm32f373xc.h" // Device headers

Queue *Usart1_RxQueue;
Queue *Usart1_TxQueue;

void init_Uart(USART_TypeDef *USART, Queue *Usart_RxQueue, Queue *Usart_TxQueue, uint32_t divider){
	if(USART == USART1){
		Usart1_RxQueue = Usart_RxQueue;
		Usart1_TxQueue = Usart_TxQueue;
	}else if(USART == USART2){
		/* Implement for USART2 functionality */
	}
	USART->CR2 &= ~(USART_CR2_LINEN | USART_CR2_CLKEN); /* Setup for asyncronous mode */
  USART->CR3 &= ~(USART_CR3_SCEN | USART_CR3_HDSEL | USART_CR3_IREN);
	USART->CR3 |= USART_CR3_CTSE | USART_CR3_RTSE;
	USART->BRR = divider;
	USART->CR1 |= USART_CR1_RXNEIE | USART_CR1_TE | USART_CR1_RE;
	USART->CR1 |= USART_CR1_UE;

}

/* Return zero if successful, non-zero if not */
int update_Baud_Rate(USART_TypeDef *USART, uint32_t baud_rate_divisor){
	USART->CR1 &= ~USART_CR1_UE; /* Disable the usart */
	USART->BRR = baud_rate_divisor; /* set new divisor */
	USART->CR1 |= USART_CR1_UE; /* reenable the usart */
	
	return 0;
}

/* In case of new data transfers, kickstart the USART device */
void send_Data(USART_TypeDef *USART){
	if(USART == USART1){
		USART1->CR1 |= USART_CR1_TXEIE;
		USART1_IRQHandler();
	}else if(USART == USART2){
		USART2->CR1 |= USART_CR1_TXEIE;
		USART2_IRQHandler();
	}
}

/* USART1 interrupt handler */ 
void USART1_IRQHandler(void){
	uint32_t interrupt =  USART1->ISR;
			
	if(interrupt & USART_ISR_TXE){
			// TX empty, send next byte
		if(queue_isEmpty(Usart1_TxQueue)){
			USART1->CR1 &= ~USART_CR1_TXEIE;
		}else{
			USART1->TDR = (uint8_t) deQueue(Usart1_TxQueue);
		}
	}
	/* Reading */
	if(interrupt & USART_ISR_RXNE){
		// RX is not empty, read it before next incoming
		enQueue(Usart1_RxQueue, USART1->RDR);
	}
}

/* USART2 interrupt handler */ 
void USART2_IRQHandler(void){
	/* Implement USART2 functionality */
}