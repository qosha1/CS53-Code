

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

void buttons_Init();
uint8_t get_Button();