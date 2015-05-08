
#include <stdint.h>

#define BUTTON_QUEUE_LENGTH	((uint32_t)0x20)
#define BUTTON_ADDRESS			((uint32_t)0x00000036)
#define BUTTON_READ					((uint16_t)0x0001)
#define BUTTON_WRITE					((uint16_t)0x0000)
#define BUTTON_KEY_REG			((uint8_t)0x02)
#define BUTTON_RESET_REG    ((uint8_t) 57)
#define BUTTON_THRESHOLD_REG0 ((uint8_t)32)
#define BUTTON_THRESHOLD_REG1 ((uint8_t)33)
#define BUTTON_THRESHOLD_REG2 ((uint8_t)34)
#define BUTTON_THRESHOLD_REG3 ((uint8_t)35)
#define BUTTON_THRESHOLD_REG4 ((uint8_t)36)
#define BUTTON_THRESHOLD_REG5 ((uint8_t)37)
#define BUTTON_THRESHOLD_REG6 ((uint8_t)38)
#define BUTTON_THRESHOLD			((uint8_t)128)

enum button_type{	// TYPES of buttons on target hardware
	ON_BUTTON,
	MEASURE_BUTTON,
	EXTRA_BUTTON1,
	EXTRA_BUTTON2,
	IILLEGAL_BUTTON,
};

enum keyCode {
	KEYCODE_ON,
	KEYCODE_MEASURE,
	KEYCODE_EXTRA1,
	KEYCODE_EXTRA2,
	KEYCODE_ILLEGAL
};

extern int32_t  Button_Initialize   (void);
int32_t Buttons_GetKey     (void);
int32_t Buttons_KeyAvailable(void);
void get_Key(void);
extern uint32_t Buttons_GetCount     (void);
///extern void EXTI4_15_IRQHandler(void);
extern void I2C1_IRQHandler(void);
