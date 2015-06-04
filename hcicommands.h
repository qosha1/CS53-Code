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

#endif // __hcicommands__


/**************************** Begin Controller and Baseband Commands ***************************/

typedef struct _tagHCI_Write_Simple_Pairing_Mode_Command_t
{
    HCI_Command_Header_t  HCI_Command_Header;
    uint8_t               Simple_Pairing_Enable;
} HCI_Write_Simple_Pairing_Mode_Command_t;


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

#define HCI_Set_Mws_Transport_Layer_Command_Size		(sizeof (HCI_Set_Mws_Transport_Layer_Command_t))
#define HCI_Set_Mws_Transport_Layer_Command_Param_Length  ((uint8_t) 0x09)
