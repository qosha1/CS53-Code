/*****< bt_control.h >***********************************************************/
/*      California Institute of Technology 									  */
/*		EECS53 project 2015   		                          			      */
/*      Portable fitness  tracker                                             */
/*                                                                            */
/*  bt_control.c - This file contains the constants and prototypes relavent to*/
/*                  the bluetooth controlling software.                       */
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


#include <stdint.h> 			//
#include "../Peripherals/Queue.h"	// Queue functions

#define BT_QUEUE_LENGTH						((uint32_t) 0x80)

#define BT_RESET_TIME							((uint32_t) 0x0010)
#define BT_STARTUP_DIVIDER 				((uint16_t) 139)
#define BT_DIVISOR								((uint16_t)   8)
#define BT_BAUD_RATE							((uint32_t) 2000000)

/* Constants for Bluetooth inquiries and connections */
#define BT_INQUIRY_LENGTH						((uint8_t) 0x30)
#define BT_NUM_INQUIRY_RESPONSES				((uint8_t) 0x00) /* Code for unlimited inquiries */

#define BT_LAP_CODE_OCTET_1						((uint8_t) 0x33)
#define BT_LAP_CODE_OCTET_2						((uint8_t) 0x8B)
#define BT_LAP_CODE_OCTET_3						((uint8_t) 0x9E)


#define HCI_VS_SET_BAUD_OPCODE				((uint16_t) 0xFF36)

extern Queue *btTxQueue;
extern Queue *btRxQueue;

void bt_Init(void);