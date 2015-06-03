#include <stdint.h> 			// 
#include "../Peripherals/Queue.h"	// Queue functions

#define BT_QUEUE_LENGTH							((uint32_t) 0x80)

#define BT_STARTUP_DIVIDER 						((uint16_t) 139)
#define BT_DIVIDER								((uint16_t)   8)
#define BT_BAUD_RATE							((uint32_t) 2000000)

/* Constants for Bluetooth inquiries and connections */
#define BT_INQUIRY_LENGTH						((uint8_t) 0x30)
#define BT_NUM_INQUIRY_RESPONSES				((uint8_t) 0x00) /* Code for unlimited inquiries */

#define BT_LAP_CODE_OCTET_1						((uint8_t) 0x33)
#define BT_LAP_CODE_OCTET_2						((uint8_t) 0x8B)
#define BT_LAP_CODE_OCTET_3						((uint8_t) 0x9E)

extern Queue *btTxQueue;
extern Queue *btRxQueue;