

#include "stdint.h"
struct _Button_Group
{
	uint8_t button1;
	uint8_t button2;
	uint8_t button3;
	uint8_t button4;
	
};

#define NO_BUTTON 	((uint8_t) 0)
#define KEY_DEBOUNCE_MS		((uint16_t) 100)

#define CAP_BUTTON1_MASK		0x01
#define CAP_BUTTON2_MASK		0x02
#define CAP_BUTTON3_MASK		0x04
#define CAP_BUTTON4_MASK		0x08
void buttons_Init();
uint8_t get_Button();