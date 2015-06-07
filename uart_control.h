/*****< uart_control.h >*******************************************************/
/*      California Institute of Technology 									  */
/*		EECS53 project 2015   		                          			      */
/*      Portable fitness  tracker                                             */
/*                                                                            */
/*  uart_control.h - This file contains the prototype function declarations   */
/*                   for those implemented in the c file                      */
/*                                                                            */
/*  Author:  Quinn Osha                                                       */
/*                                                                            */
/*** MODIFICATION HISTORY *****************************************************/
/*                                                                            */
/*   mm/dd/yy  F. Lastname    Description of Modification                     */
/*   --------  -----------    ------------------------------------------------*/
/*   05/27/15  Q. Osha        Initial revision.                               */
/*   05/30/15  Q. Osha        Seperated USART functions out of bluetooth      */
 /*                           functionality                                   */
/******************************************************************************/

#include "../Peripherals/Queue.h"
#include "stm32f373xc.h" // Device header

/* Initialize the given USART with a receiving queue and a transmitting queue at 
 * a certain baud rate given by the baud rate divider */
void init_Uart(USART_TypeDef *USART, Queue *Usart_RxQueue, Queue *Usart_TxQueue, 
								uint32_t divider);
/* Change the baud rate of an available USART peripheral. */
int update_Baud_Rate(USART_TypeDef *USART, uint32_t baud_rate_divisor);
void send_Data(USART_TypeDef *USART);

/* Interrupt handlers for USART devices */
extern void USART1_IRQHandler(void);
extern void USART2_IRQHandler(void);