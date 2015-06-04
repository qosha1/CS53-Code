
#include "bt_control.h"   // bluetooth specific constants
#include "HCI_constants.h" //
#include "uart_control.h" // uart specific headers
#include "stm32f373xc.h" // Device header
#include "../Main/mpu_constants.h" // mpu specific constants
#include "../Peripherals/i2c_control.h"
#include "hci_api.h"
//#include "HCITypes.h"
#include <stdlib.h>

Queue *btTxQueue; // Queues for handling input/output of data to controller
Queue *btRxQueue;
HCI_Bt_Inst_t *bt_instance; /* structure for the local bluetooth instance */

/* function prototypes */
void init_Bt_Controller();
void get_Bt_Response(void *hci_struct, uint8_t hci_header_size, uint8_t hci_parameter_size);
uint8_t enqueue_Bt_Command(void *bt_command, uint8_t command_size);
uint8_t set_Bt_Scan_Enable();
uint8_t set_Simple_Pairing();

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
	/* Initialize memory for the bluetooth instance */
	bt_instance = malloc(HCI_BT_INST_SIZE);

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
	uint8_t event_byte;
	uint8_t event_value;
	HCI_Set_Mws_Transport_Layer_Command_t *set_baud_command = malloc(HCI_Set_Mws_Transport_Layer_Command_Size);
	set_baud_command->HCI_Command_Header->Command_OpCode = HCI_COMMAND_OPCODE_SET_MWS_TRANSPORT_LAYER;
	set_baud_command->HCI_Command_Header->Parameter_Total_Length = HCI_Set_Mws_Transport_Layer_Command_Param_Length;
	set_baud_command->From_Mws_Baud_Rate = BT_BAUD_RATE;
	set_baud_command->To_Mws_Baud_Rate 	= BT_BAUD_RATE;
	// Add command to send queue
	error = enqueue_Bt_Command(set_baud_command, HCI_Set_Mws_Transport_Layer_Command_Size);
	if(!error){ /* If no error, send command */
		send_Data(BT_USART);
	}
	
	HCI_Event_Packet_Header_t *command_response = malloc(HCI_EVENT_PACKET_HEADER_SIZE);
	command_response->HCIPacketData = malloc(HCI_SET_MWS_TRANSPORT_LAYER_RESPONSE_PARAM_SIZE);	
	/*wait for command complete event */
	get_Bt_Response(command_response, HCI_EVENT_PACKET_HEADER_SIZE, HCI_SET_MWS_TRANSPORT_LAYER_RESPONSE_PARAM_SIZE);

	if(command_response->HCIPacketType == ptHCIEventPacket && 
		command_response->HCIEventCode == HCI_EVENT_CODE_COMMAND_COMPLETE){
		/* The response is the correct command complete.  */
		update_Baud_Rate(BT_USART, BT_DIVISOR); /* update the USART baud rate */
	}
    free(set_baud_command);
    /* Set other bluetooth controller attributes */
    set_Bt_Scan_Enable();
    set_Simple_Pairing();
}

/* Returns zero (0) if successful, non-zero if not */
uint8_t get_Bt_Response(HCI_Event_Data_t *hci_response_struct){
	uint8_t response_byte;
	uint8_t response_value; /* next octet received from BT_controller */
	uint8_t *hci_response = (uint8_t *)hci_struct;
	uint16_t response_length;  /* How many octets there are after the header */
	HCI_Packet_Type_t response_type; /* Type of response that is being decoded */
    
	response_type = deQueue(btRxQueue);
	if(response_type == ptHCISCODataPacket || ptHCIACLDataPacket){
		/* TODO: handle data packets */
	}else if(response_type == ptHCIEventPacket){ /* Get event packet */ 
		/*  Build the header */
        hci_response_struct->Event_Data_Type = (HCI_Event_Type_t) deQueue(btRxQueue);
		hci_response_struct->Event_Data_Size = (uint8_t) deQueue(btRxQueue);
        
		/* Build the parameter array */
		hci_response = (uint8_t *)hci_response; /* pointer to pointer to data */
		for(repsonse_byte = 0; response_byte < hci_response_struct->Event_Data_Size; response_byte++){
			while(queue_isEmpty(btRxQueue)){ }
			response_value = deQueue(btRxQueue);
			*hci_response = response_value;
			hci_response++;
		}
	}
    return 0;
}

/* Returns Zero (0) if successful, nonzero if not */ 
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
uint8_t set_Bt_Scan_Enable(){
    uint8_t error;
    HCI_Write_Scan_Enable_Command_t *set_scan_command = malloc(HCI_WRITE_SCAN_ENABLE_COMMAND_SIZE);
    set_scan_command->HCI_Command_Header->Command_OpCode = HCI_COMMAND_OPCODE_WRITE_SCAN_ENABLE;
    set_scan_command->HCI_Command_Header->Parameter_Total_Length =
                                        HCI_WRITE_SCAN_ENABLE_COMMAND_SIZE - HCI_COMMAND_HEADER_SIZE;
    set_scan_command->Scan_Enable = HCI_SCAN_ENABLE_INQUIRY_SCAN_ENABLED_PAGE_SCAN_ENABLED;
    error = enqueue_Bt_Command(set_scan_command, HCI_WRITE_SCAN_ENABLE_COMMAND_SIZE);
    
    free(set_scan_command);
    if(!error){
        return 0;
    }
    return 1;
    
}
uint8_t set_Simple_Pairing(){
    uint8_t error;
    HCI_Write_Simple_Pairing_Mode_Command_t *set_pairing_mode = malloc(HCI_SIMPLE_PAIRING_MODE_COMMAND_SIZE);
    
    set_pairing_mode->HCI_Command_Header->Command_OpCode = HCI_COMMAND_OPCODE_WRITE_SIMPLE_PAIRING_MODE;
    set_pairing_mode->HCI_Command_Header->Parameter_Total_Length =
                                            HCI_SIMPLE_PAIRING_MODE_COMMAND_SIZE - HCI_COMMAND_HEADER_SIZE;
    set_pairing_mode->Simple_Pairing_Enable = HCI_SIMPLE_PAIRING_MODE_ENABLED;
    error = enqueue_Bt_Command(set_pairing_mode);
    
    free(set_pairing_mode);
    if(!error){
        return 0;
    }
    return 1;
}

void connect_Bluetooth(){
    
    /* Process:
    	1. Start inquiry
    	2. Acquire information (BD_ADDR of other controller)
    	3. Page (connect) to other 
    */
    bt_instance->LAP->octet_1 = BT_LAP_CODE_OCTET_1;
    bt_instance->LAP->octet_2 = BT_LAP_CODE_OCTET_2;
    bt_instance->LAP->octet_3 = BT_LAP_CODE_OCTET_3;
    HCI_Event_Data_t *event_response = malloc(HCI_EVENT_DATA_SIZE);
    start_Bt_Inquiry(event_response);
    
}

void start_Bt_Inquiry(HCI_Event_Data_t *event_response){
	uint8_t error;
	HCI_Inquiry_Command_t *start_inquiry = malloc(HCI_Inquiry_Command_Size);
	
    start_inquiry->HCI_Command_Header->Command_OpCode = HCI_COMMAND_OPCODE_INQUIRY;
	start_inquiry->HCI_Command_Header->Parameter_Total_Length = 
									HCI_INQUIRY_COMMAND_SIZE - HCI_COMMAND_HEADER_SIZE;
	start_inquiry->LAP 			  = bt_instance->LAP;
	start_inquiry->Inquiry_Length = BT_INQUIRY_LENGTH;
	start_inquiry->Num_Responses  = BT_NUM_INQUIRY_RESPONSES;
	error = enqueue_Bt_Command(start_inquiry, HCI_INQUIRY_COMMAND_SIZE);
    free(start_inquiry);
    
	if(!error){
        error = get_Bt_Response(event_reponse);
        if (!error) {
            return 0;
        }
	}
    return 1;
}



