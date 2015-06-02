#ifndef __HCI_APIH__
#define __HCI_APIH__

#include "HCI_constants.h"  /* HCI data type constants */

#define HEADER_SIZE 	2

typedef struct _tagHCI_Event_Packet_Header_t
{
   HCI_PacketType_t HCIPacketType;
	 uint8_t					HCIEventCode;
   uint8_t		      HCIPacketLength;
   uint8_t		      *HCIPacketData;
} HCI_Event_Packet_Header_t;

#define HCI_EVENT_PACKET_HEADER_SIZE                    (sizeof(HCI_Event_Packet_Header_t))

#define HCI_SET_MWS_TRANSPORT_LAYER_RESPONSE_PARAM_SIZE (uint8_t (0x01))
/*
typedef struct _tagHCI_Set_Transport_Layer_Complete_Event_Data_t
{
   uint8_t 	Num_Command_Packets;
   uint16_t Command_Opcode;
	 
} HCI_Set_Transport_Layer_Complete_Event_Data_t;

#define HCI_SET_TRANSPORT_LAYER_COMPLETE_EVENT_DATA_SIZE            (sizeof(HCI_Set_Transport_Layer_Complete_Event_Data_t))
*/
typedef struct _tagHCI_Set_Mws_Transport_Layer_Command_t
{
   uint16_t HCI_Command_Header;
   uint8_t  HCI_Command_Parameter_Length;
	 uint8_t  Transport_Layer;
	 uint32_t To_Mws_Baud_Rate;
	 uint32_t From_Mws_Baud_Rate;
} HCI_Set_Mws_Transport_Layer_Command_t;

#define HCI_Set_Mws_Transport_Layer_Command_Size		(sizeof (HCI_Set_Mws_Transport_Layer_Command_t))
#define HCI_Set_Mws_Transport_Layer_Command_Param_Length  ((uint8_t) 0x09)
#endif // __HCI_API__