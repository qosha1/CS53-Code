/*****< hci_commands.h >********************************************************/
/*      California Institute of Technology 									  */
/*		EECS53 project 2015   		                          			      */
/*      Portable fitness  tracker                                             */
/*                                                                            */
/*  hci_commands.h - This file contains stucture definitions for the commands */
/*                   in the HCI transport layer that are implemented. This is */
/*                   by no means a comprehensive list nor is it supposed to be*/
/*                   Each command, its parameters and its responses is detailed*/
/*                   in the Bluetooht v4.0 specification.                     */
/*                                                                            */
/*  Author:  Quinn Osha                                                       */
/*                                                                            */
/*** MODIFICATION HISTORY *****************************************************/
/*                                                                            */
/*   mm/dd/yy  F. Lastname    Description of Modification                     */
/*   --------  -----------    ------------------------------------------------*/
/*   05/14/15  Q. Osha        Initial revision (in hciapi file)               */
/*   06/02/15  Q. Osha        Moved these structures to own file              */
/******************************************************************************/


#ifndef __hcicommands__
#define __hcicommands__

#include "bt_types.h"

typedef struct _tagHCI_Command_Header_t
{
   uint16_t Command_OpCode;
   uint8_t  Parameter_Total_Length;
}  HCI_Command_Header_t;

#define HCI_COMMAND_HEADER_SIZE                          (sizeof(HCI_Command_Header_t))

#define HCI_COMMAND_MAX_SIZE                             (sizeof(HCI_Command_Header_t) + (sizeof(Byte_t)*256))


/******************************** Begin Link Control Commands **********************************/

typedef struct _tagHCI_Inquiry_Command_t
{
   HCI_Command_Header_t HCI_Command_Header;
   LAP_t                LAP;
   uint8_t              Inquiry_Length;
   uint8_t              Num_Responses;
}  HCI_Inquiry_Command_t;

#define HCI_INQUIRY_COMMAND_SIZE                         (sizeof(HCI_Inquiry_Command_t))


/**************************** Begin VS_ Controller and Baseband Commands ***************************/
typedef struct _tagHCI_Vs_Set_Baud_Command_t
{
    HCI_Command_Header_t HCI_Command_Header;
    uint32_t Baud_Rate;
} HCI_Vs_Set_Baud_Command_t;

#define HCI_VS_SET_BAUD_COMMAND_SIZE		(sizeof (HCI_Vs_Set_Baud_Command_t))

/**************************** Begin Controller and Baseband Commands ***************************/

typedef struct _tagHCI_Write_Simple_Pairing_Mode_Command_t
{
    HCI_Command_Header_t  HCI_Command_Header;
    uint8_t               Simple_Pairing_Enable;
} HCI_Write_Simple_Pairing_Mode_Command_t;

#define HCI_WRITE_SIMPLE_PAIRING_MODE_COMMAND_SIZE       (sizeof(HCI_Write_Simple_Pairing_Mode_Command_t))

typedef struct _tagHCI_Write_Scan_Enable_Command_t
{
    HCI_Command_Header_t    HCI_Command_Header;
    uint8_t                 Scan_Enable;
}  HCI_Write_Scan_Enable_Command_t;

#define HCI_WRITE_SCAN_ENABLE_COMMAND_SIZE               (sizeof(HCI_Write_Scan_Enable_Command_t))

typedef struct _tagHCI_Set_Mws_Transport_Layer_Command_t
{
    HCI_Command_Header_t HCI_Command_Header;
    uint8_t  Transport_Layer;
    uint32_t To_Mws_Baud_Rate;
    uint32_t From_Mws_Baud_Rate;
} HCI_Set_Mws_Transport_Layer_Command_t;

#define HCI_SET_MWS_TRANSPORT_LAYER_COMMAND_SIZE		(sizeof (HCI_Set_Mws_Transport_Layer_Command_t))








#endif // __hcicommands__