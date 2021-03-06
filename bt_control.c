/*****< bt_control.c >***********************************************************/
/*      California Institute of Technology 									  */
/*		EECS53 project 2015   		                          			      */
/*      Portable fitness  tracker                                             */
/*                                                                            */
/*  bt_control.c - This file contains functions that control the bluetooth 	  */
/*  				controller-to-host HCI communication. It is the host side */
/*					initialization and control functionality, including the   */
/*					GPIOs.					               					  */
/*                                                                            */
/*  Author:  Quinn Osha                                                       */
/*                                                                            */
/*** MODIFICATION HISTORY *****************************************************/
/*                                                                            */
/*   mm/dd/yy  F. Lastname    Description of Modification                     */
/*   --------  -----------    ------------------------------------------------*/
/*   05/10/15  Q. Osha        Initial revision.                               */
/*   05/25/15  Q. Osha        Separate UART functions from the bluetooth 	  */
/*								functions. 		                              */
/*	 06/01/15  Q. Osha		  Add further functionality for bluetooth, init   */
/*								functions. 									  */
/******************************************************************************/


#include "bt_control.h"   // bluetooth specific constants
#include "HCI_constants.h" //
#include "hci_api.h"
#include "hcicommands.h"  // BT Command structures and constants
#include "uart_control.h" // uart specific headers
#include "stm32f373xc.h" // Device header
#include "../Main/mpu_constants.h" // mpu specific constants
#include "../Peripherals/i2c_control.h"
#include <stdlib.h>
#include "ti_stupid_sp.h" //Need to include TI's incredibly stupid (16KB!) service
                        //pack to even make the bluetooth chip turn on. No, if
                        // you were wondering, its not mentioned anywhere in the
                        // datasheet. Instead you must search through their Wiki
                        // page to finally find that it is a requirement.
													
Queue *btTxQueue; // Queues for handling input/output of data to controller
Queue *btRxQueue;
HCI_Bt_Inst_t *bt_instance; /* structure for the local bluetooth instance */

/* function prototypes */
void init_Bt_Controller(void);
uint8_t get_Bt_Response(HCI_Event_Data_t *hci_response_struct);
uint8_t enqueue_Bt_Command(void *bt_command);
uint8_t set_Bt_Scan_Enable(void);
uint8_t set_Simple_Pairing(void);
void start_Bt_Inquiry(HCI_Event_Data_t *event_response);
void fix_TIs_Stupid_Mistakes();



/*  Function: bt_Init()
 /**
 * @brief: This function initializes the GPIOs used for communication with the 
 *          bluetooth controller. The USART pins are configured and set to 
 *          the appropriate alternate functions.
 *
 * @param: none
 * @retval: none
 */
void bt_Init(){
	RCC->AHBENR	|= RCC_AHBENR_GPIOAEN; 			/* Enable GPIOA clock*/
	// Enable I2C clocks 
	RCC->APB2ENR   |= RCC_APB2ENR_USART1EN;  	/* Enable USART1 clock*/

	/* Initialize the USART bus GPIO pins    */
	GPIOA->AFR[1]  |= (((uint32_t) BT_USART_AF) << 4 * (BT_HCI_RX % 8));// alternate function setup
	GPIOA->AFR[1]  |= (((uint32_t) BT_USART_AF) << 4 * (BT_HCI_TX % 8));// alternate function setup
	GPIOA->AFR[1]  |= (((uint32_t) BT_USART_AF) << 4 * (BT_HCI_CTS % 8));// alternate function setup
	GPIOA->AFR[1]  |= (((uint32_t) BT_USART_AF) << 4 * (BT_HCI_RTS % 8));// alternate function setup

	GPIOA->PUPDR  |= (1UL << 2*BT_HCI_TX);		//  pull up
	GPIOA->PUPDR  |= (1UL << 2*BT_HCI_RTS);	//  pull up
    
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
	
	Delay(BT_RESET_TIME); /* wait for bluetooth reset */
	GPIOA->ODR		  |= (1UL << BT_SHUTD); /* release shutdown pin */
	Delay(BT_RESET_TIME);
	fix_TIs_Stupid_Mistakes();
	/* setup the bluetooth controller */
	init_Bt_Controller();
}




/*
 /**
 * @brief   This function initializes specific attributes of the external TI 
 *          Bluetooth controller. The controller starts at a low baud rate and 
 *          it is suggested to increase it after boot up.
 *
 *          Bt controller defaults:
 *              bit rate: 115.2 kbps
 *              8 bits
 *              1 stop bit
 *              No parity bit
 *
 * @param   none
 * @retval  none
 */
void init_Bt_Controller(){
	// Reset baud rate
	// 
	uint8_t error;
	uint8_t event_byte;
	uint8_t event_value;
	HCI_Vs_Set_Baud_Command_t *set_baud_command = malloc(HCI_VS_SET_BAUD_COMMAND_SIZE);
	set_baud_command->HCI_Command_Header.Command_OpCode = HCI_VS_SET_BAUD_OPCODE;
	set_baud_command->HCI_Command_Header.Parameter_Total_Length = 4;
	set_baud_command->Baud_Rate = BT_BAUD_RATE;
	
	// Add command to send queue
	error = enqueue_Bt_Command(set_baud_command);
	if(!error){ /* If no error, send command */
		send_Data(BT_USART);
	}
	HCI_Event_Data_t *hci_response = malloc(HCI_EVENT_DATA_SIZE);
	/*wait for command complete event */
	get_Bt_Response(hci_response);
	

	if(hci_response->Event_Data_Type == HCI_EVENT_CODE_COMMAND_COMPLETE){
		/* The response is the correct command complete.  */
		update_Baud_Rate(BT_USART, BT_DIVISOR); /* update the USART baud rate */
	}
    free(set_baud_command);
    /* Set other bluetooth controller attributes */
    set_Bt_Scan_Enable();
    set_Simple_Pairing();
}

/* Function Description:
 /**
 * @brief   This function handles organizing the byte-wise incoming BT data
 *          on the USART and turn it into the BT events and codes that are 
 *          used in the 4-wire HCI communication protocol.
 *
 * @param   A memory-allocated HCI_Event_Data_t struct to store the results 
 *          of any BT responses that are currently in the btRxQueue.
 * @retval  Returns zero (0) if successful, non-zero if not
 */
uint8_t get_Bt_Response(HCI_Event_Data_t *hci_response_struct){
	uint8_t response_byte;
	uint8_t response_value; /* next octet received from BT_controller */
	uint16_t response_length;  /* How many octets there are after the header */
	uint8_t *hci_response;
	HCI_PacketType_t response_type; /* Type of response that is being decoded */
  
	while(queue_isEmpty(btRxQueue)){ }  
	response_type = (HCI_PacketType_t) deQueue(btRxQueue);
	if(response_type == ptHCISCODataPacket || response_type == ptHCIACLDataPacket){
		/* TODO: handle data packets */
	}else if(response_type == ptHCIEventPacket){ /* Get event packet */ 
		/*  Build the header */
    while(queue_isEmpty(btRxQueue)){ }
		hci_response_struct->Event_Data_Type = (HCI_Event_Type_t) deQueue(btRxQueue);
		while(queue_isEmpty(btRxQueue)){ }
		hci_response_struct->Event_Data_Size = (uint8_t) deQueue(btRxQueue);
       
		/* Build the parameter array */
		hci_response = (uint8_t *) (&hci_response_struct->Event_Data); /* pointer to pointer to data */
		for(response_byte = 0; response_byte < hci_response_struct->Event_Data_Size; response_byte++){
			while(queue_isEmpty(btRxQueue)){ }
			response_value = deQueue(btRxQueue);
			*hci_response = response_value;
			hci_response++;
		}
	}
    return 0;
}

/* Function Description:
 /**
 * @brief   This function handles breaking down Bluetooth commands into their
 *          base byte-wise orginization and adding them to the queue to be sent
 *          when possible.
 *
 * @param   A configured BT HCI command that follows the structuring the
 *          bt 4.0 protocol.
 * @retval  Returns zero (0) if successful, non-zero if not
 */
uint8_t enqueue_Bt_Command(void *bt_command){
	uint8_t i;
	uint8_t *command = (uint8_t *)bt_command;
	while(queue_isFull(btTxQueue)){ }
	enQueue(btTxQueue, ptHCICommandPacket);
	while(queue_isFull(btTxQueue)){ }
	enQueue(btTxQueue, *command);
	command++;
	while(queue_isFull(btTxQueue)){ }
	enQueue(btTxQueue, *command);
	command++;
	/* Third parameter is the total length */
	i = *command;
	while(queue_isFull(btTxQueue)){ }
	enQueue(btTxQueue, *command);
	command++;
	while(i > 0){
		while(queue_isFull(btTxQueue)){ }
		enQueue(btTxQueue, *command);
		command++;
		i--;
	}
	return i;
}

/* Function Description:
 /**
 * @brief   This function builds a HCI_Write_Scan_Enable_Command_t structure to tell
 *          the offboard controller to enable scanning for potential connections. This
 *          command has no parameters.
 *
 * @param   none
 * @retval  Returns zero (0) if successful, non-zero if not
 */
uint8_t set_Bt_Scan_Enable(){
    uint8_t error;
    HCI_Write_Scan_Enable_Command_t *set_scan_command = malloc(HCI_WRITE_SCAN_ENABLE_COMMAND_SIZE);
    set_scan_command->HCI_Command_Header.Command_OpCode = HCI_COMMAND_OPCODE_WRITE_SCAN_ENABLE;
    set_scan_command->HCI_Command_Header.Parameter_Total_Length =
                                        HCI_WRITE_SCAN_ENABLE_COMMAND_SIZE - HCI_COMMAND_HEADER_SIZE;
    set_scan_command->Scan_Enable = HCI_SCAN_ENABLE_INQUIRY_SCAN_ENABLED_PAGE_SCAN_ENABLED;
    error = enqueue_Bt_Command(set_scan_command);
    
    free(set_scan_command);
    if(!error){
        return 0;
    }
    return 1;
    
}

/* Function Description:
/**
 * @brief   This function builds a HCI_Write_Simple_Pairing_Mode_Command_t structure for 
 *          enabling the Bluetooth Simple Secure Pairing mode in the controller. No 
 *          parameters for this HCI command are possible.
 *
 * @param   none
 * @retval  Returns zero (0) if successful, non-zero if not
 */
uint8_t set_Simple_Pairing(){
    uint8_t error;
    HCI_Write_Simple_Pairing_Mode_Command_t *set_pairing_mode = malloc(HCI_WRITE_SIMPLE_PAIRING_MODE_COMMAND_SIZE);
    
    set_pairing_mode->HCI_Command_Header.Command_OpCode = HCI_COMMAND_OPCODE_WRITE_SIMPLE_PAIRING_MODE;
    set_pairing_mode->HCI_Command_Header.Parameter_Total_Length =
                                            HCI_WRITE_SIMPLE_PAIRING_MODE_COMMAND_SIZE - HCI_COMMAND_HEADER_SIZE;
    set_pairing_mode->Simple_Pairing_Enable = HCI_SIMPLE_PAIRING_MODE_ENABLED;
    error = enqueue_Bt_Command(set_pairing_mode);
    
    free(set_pairing_mode);
    if(!error){
        return 0;
    }
    return 1;
}

/* Function Description:
 /**
 * @brief   This function is incomplete. It will be used to handle the various minor command
 *          necessary to generate a connection with another bluetooth controller.
 *
 * @param   none
 * @retval  none
 */
void connect_Bluetooth(){
    
    /* Process:
    	1. Start inquiry
    	2. Acquire information (BD_ADDR of other controller)
    	3. Page (connect) to other 
    */
    bt_instance->LAP.octet_1 = BT_LAP_CODE_OCTET_1;
    bt_instance->LAP.octet_2 = BT_LAP_CODE_OCTET_2;
    bt_instance->LAP.octet_3 = BT_LAP_CODE_OCTET_3;
    HCI_Event_Data_t *event_response = malloc(HCI_EVENT_DATA_SIZE);
    start_Bt_Inquiry(event_response);
    
}

/* Function Description:
 /**
 * @brief   This function builds a HCI_Inquiry_Command_t structure to generate
 *          a bluetooth inquiry.
 *
 * @param   an HCI_Event_Data_t struct to place the controller response in.
 * @retval  Returns zero (0) if successful, non-zero if not
 */
void start_Bt_Inquiry(HCI_Event_Data_t *event_response){
	uint8_t error;
	HCI_Inquiry_Command_t *start_inquiry = malloc(HCI_INQUIRY_COMMAND_SIZE);
	
  start_inquiry->HCI_Command_Header.Command_OpCode = HCI_COMMAND_OPCODE_INQUIRY;
	start_inquiry->HCI_Command_Header.Parameter_Total_Length = 
									HCI_INQUIRY_COMMAND_SIZE - HCI_COMMAND_HEADER_SIZE;
	start_inquiry->LAP 			  = bt_instance->LAP;
	start_inquiry->Inquiry_Length = BT_INQUIRY_LENGTH;
	start_inquiry->Num_Responses  = BT_NUM_INQUIRY_RESPONSES;
	error = enqueue_Bt_Command(start_inquiry);
    free(start_inquiry);
    
	if(!error){
        error = get_Bt_Response(event_response);
        if (!error) {
           
        }
	}
}

/* Here is where we get to fix all of TI's mistakes by 
 * loading their service pack. Woo.
*/
/* Function Description:
 /**
 * @brief   Here is where we get to fix all of TI's mistakes by
 *          loading their service pack. Woo. This function
 *          uses the ti_stupd_sp.h file to patch the controller
 *          firmware upon startup. This is a necessary precursor to
 *          enabling the BT controller. 
 *
 * @param   none
 * @retval  none
 */
void fix_TIs_Stupid_Mistakes(void){
	uint32_t current_mistake;
	uint32_t num_bytes_needed_to_fix_ti_mistakes = sizeof(BasePatch);
	
	for(current_mistake = 0; current_mistake < num_bytes_needed_to_fix_ti_mistakes; current_mistake++){
		while(queue_isFull(btTxQueue)){ }
		enQueue(btTxQueue, BasePatch[current_mistake]);
		if(queue_isFull(btTxQueue)){ 
			send_Data(BT_USART);
		}
		
	}
}


