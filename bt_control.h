#include <stdint.h> 			// 
#include "../Peripherals/Queue.h"	// Queue functions

#define BT_QUEUE_LENGTH								((uint32_t) 0x80)

#define BT_STARTUP_DIVIDER 						((uint16_t) 139)
#define BT_DIVIDER										((uint16_t)   8)
#define BT_BAUD_RATE									((uint32_t) 2000000)

#define BT_COMMAND_PACKET_TYPE 				((uint8_t) 01)
#define BT_ACL_DATA_PACKET_TYPE 			((uint8_t) 02)
#define BT_SCO_DATA_PACKET_TYPE				((uint8_t) 03)
#define BT_EVENT_PACKET_TYPE 					((uint8_t) 04)

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
extern Queue *btTxQueue;
extern Queue *btRxQueue;