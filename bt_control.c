
#include "bt_control.h"   // bluetooth specific constants
#include "stm32f373xc.h" // Device header
#include "../Main/mpu_constants.h" // mpu specific constants
#include "../Peripherals/i2c_control.h"
#include <stdlib.h>

/* function prototypes */
void init_Uart();
void init_Bt_Controller();

void bt_init(){
	
	RCC->AHBENR	|= RCC_AHBENR_GPIOAEN; 			/* Enable GPIOA clock*/
	// Enable I2C clocks 
	RCC->APB2ENR   |= RCC_APB2ENR_USART1EN;  	/* Enable USART1 clock*/

	/* Initialize the USART bus GPIO pins    */
	GPIOA->AFR[1]  |= (((uint32_t) BT_USART_AF) << 4 * (BT_HCI_RX % 8));// alternate function setup
	GPIOA->AFR[1]  |= (((uint32_t) BT_USART_AF) << 4 * (BT_HCI_TX % 8));// alternate function setup
	GPIOA->AFR[1]  |= (((uint32_t) BT_USART_AF) << 4 * (BT_HCI_CTS % 8));// alternate function setup
	GPIOA->AFR[1]  |= (((uint32_t) BT_USART_AF) << 4 * (BT_HCI_RTS % 8));// alternate function setup
		
	GPIOA->OTYPER  |= (1UL << 1*BT_HCI_RX);		//  open drain
	GPIOA->OTYPER  |= (1UL << 1*BT_HCI_TX);		//  open drain
	GPIOA->OTYPER  |= (1UL << 1*BT_HCI_CTS);	//  open drain
	GPIOA->OTYPER  |= (1UL << 1*BT_HCI_RTS);	//  open drain
		
	GPIOA->OSPEEDR |= (3UL << 2*BT_HCI_RX);		//  high speed
	GPIOA->OSPEEDR |= (3UL << 2*BT_HCI_TX);		//  high speed
	GPIOA->OSPEEDR |= (3UL << 2*BT_HCI_CTS);	//  high speed
	GPIOA->OSPEEDR |= (3UL << 2*BT_HCI_RTS);	//  high speed
	
	GPIOA->MODER	 |= (2UL << 2*BT_HCI_RX)  ; /* SET TO ALTERNATE FUNCTION	*/
	GPIOA->MODER 	 |= (2UL << 2*BT_HCI_TX)  ; /* SET TO ALTERNATE FUNCTION */
	GPIOA->MODER	 |= (2UL << 2*BT_HCI_CTS) ; /* SET TO ALTERNATE FUNCTION	*/
	GPIOA->MODER 	 |= (2UL << 2*BT_HCI_RTS) ; /* SET TO ALTERNATE FUNCTION */

	//Bluetooth shutdown output
	GPIOA->MODER    |=  (1UL << 2*BT_SHUTD)   ;   /* output             */
  GPIOA->OSPEEDR  &= ~((3UL << 2*BT_SHUTD)  );  /* Low Speed          */
  GPIOA->OTYPER   |=  (1UL << 1*BT_SHUTD);
	GPIOA->PUPDR    |=  (1UL << 2*BT_SHUTD)   ;   /* OD, pullup         */
	
	init_Uart();
	
	Delay(20); /* wait for bluetooth reset */
	GPIOA->ODR		  |= (1UL << BT_SHUTD); /* release shutdown pin */
	
	/* setup the bluetooth controller */
	init_Bt_Controller();
}

void init_Uart(){
	BT_USART->CR1 |= USART_CR1_TXEIE | USART_CR1_RXNEIE | USART_CR1_TE | USART_CR1_RE;
	BT_USART->CR2 &= ~(USART_CR2_LINEN | USART_CR2_CLKEN); /* Setup for asyncronous mode */
  BT_USART->CR3 &= ~(USART_CR3_SCEN | USART_CR3_HDSEL | USART_CR3_IREN);
	BT_USART->CR3 |= USART_CR3_CTSE | USART_CR3_RTSE;
	BT_USART->BRR = BT_STARTUP_DIVIDER;
	BT_USART->CR1 |= USART_CR1_UE;
}

/* Bt controller defaults:
  bit rate: 115.2 kbps
	8 bits
	1 stop bit
	No parity bit
*/
void init_Bt_Controller(){
	
}
