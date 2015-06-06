
#ifndef MPU_CONSTANTS
#define MPU_CONSTANTS
/* general definitions */
	
typedef enum
{
	true = 1,
	false = 0
	}boolean;

				/* GPIO pin definitions */ 

// PA BLOCK
typedef enum
	{
	CAP_BUTTON1 = 0,
	CAP_BUTTON2 = 1,
	CAP_BUTTON3 = 2,
	CAP_BUTTON4 = 3,
	BT_SHUTD 		= 8,
	BT_HCI_RX   = 9,
	BT_HCI_TX   = 10,
	BT_HCI_RTS  = 11,
	BT_HCI_CTS  = 12
}PA_Block;

// PB Block
typedef enum
{
	MPU_INTRPT	= 5,
	MPU_SCL			= 6,
	MPU_SDA			= 7,
	MPU_FSYYNC	= 9
}PB_Block;
	
// PC Block
typedef enum
{
	DISPLAY_RST =  8,
	DISPLAY_RW  =	10,
	DISPLAY_ERD = 11,
	DISPLAY_DC  = 12
}PC_Block;

// PF Block
typedef enum
{
	DISPLAY_SCL  = 6,
  DISPLAY_SDA  = 7
}PF_Block;

/*							 Alternate function declarations				*/ 

#define BT_USART_AF 		7
#define CAP_BUTTON_AF		3
#define DISPLAY_I2C_AF	4
#define MPU_I2C_AF			4
	
#define MPU_I2C			I2C1
#define DISPLAY_I2C	I2C2
	
#define BT_USART 		USART1


#define MS_PER_SAMPLE			1000



#endif // MPU_CONSTANTS
