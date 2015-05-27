
#include "bt_control.h"   // bluetooth specific constants
#include "HCI_constants.h" //
#include "uart_control.h" // uart specific headers
#include "stm32f373xc.h" // Device header
#include "../Main/mpu_constants.h" // mpu specific constants
#include "../Peripherals/i2c_control.h"
//#include "HCIAPI.h"
//#include "HCITypes.h"
#include <stdlib.h>

Queue *btTxQueue; // Queues for handling input/output of data to controller
Queue *btRxQueue;

/* function prototypes */
void init_Bt_Controller();
uint8_t enqueue_Bt_Command(void *bt_command, uint8_t command_size);

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
	
		/* Setup the queues for data transfer */	
	btTxQueue = initQueue(BT_QUEUE_LENGTH);
	btRxQueue = initQueue(BT_QUEUE_LENGTH);
	
	init_Uart(BT_USART, btRxQueue, btTxQueue, BT_STARTUP_DIVIDER);
	
	Delay(20); /* wait for bluetooth reset */
	GPIOA->ODR		  |= (1UL << BT_SHUTD); /* release shutdown pin */
	
	/* setup the bluetooth controller */
	init_Bt_Controller();
}


/* Bt controller defaults:
  bit rate: 115.2 kbps
	8 bits
	1 stop bit
	No parity bit
*/
void init_Bt_Controller(){
	// Reset baud rate
	// 
	uint8_t error;
	HCI_Set_Mws_Transport_Layer_Command_t *set_baud_command = malloc(HCI_Set_Mws_Transport_Layer_Command_Size);
	set_baud_command->HCI_Command_Header = HCI_COMMAND_OPCODE_SET_MWS_TRANSPORT_LAYER;
	set_baud_command->HCI_Command_Parameter_Length = HCI_Set_Mws_Transport_Layer_Command_Param_Length;
	set_baud_command->From_Mws_Baud_Rate = BT_BAUD_RATE;
	set_baud_command->To_Mws_Baud_Rate 	= BT_BAUD_RATE;
	// Add command to send queue
	error = enqueue_Bt_Command(set_baud_command, HCI_Set_Mws_Transport_Layer_Command_Size);
	if(!error){ /* If no error, send command */
		send_Data(BT_USART);
	}
}

/* Returns Zero (0) if successful */ 
uint8_t enqueue_Bt_Command(void *bt_command, uint8_t command_size){
	uint8_t i = command_size;
	uint8_t *command = (uint8_t *)bt_command;
	while(i > 0 && !queue_isFull(btTxQueue)){
		enQueue(btTxQueue, *command);
		command++;
		i++;
	}
	return i;
}
	
void start_Bt_Inquiry(){
	
}
void connect_Bluetooth(){
    
}


