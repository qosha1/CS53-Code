#include <stdint.h>
#include "stm32f373xc.h"                  // Device header
#include "Queue.h"


#define I2C_TIMING_PRESC			((uint32_t)0x00000001)
#define I2C_TIMING_SCLDEL			((uint32_t)0x00000004)
#define I2C_TIMING_SDADEL			((uint32_t)0x00000002)
#define I2C_TIMING_SCLH				((uint32_t)0x0000000F)
#define I2C_TIMING_SCLL				((uint32_t)0x00000013)

// display event queue
#define DISPLAY_QUEUE_LENGTH	((uint32_t)0x00000100)
#define DISPLAY_RESET_TIME		((uint32_t)0x00000100)

#define DISPLAY_ADDRESS 			((uint32_t)0x00000078)
#define DISPLAY_DATA_ADDRESS 	((uint32_t)0x0000007A)
#define DISPLAY_WIDTH					((uint16_t)0x0100)
#define DISPLAY_HEIGHT				((uint16_t)0x0008)
#define DISPLAY_ON 						((uint8_t)0xAF)
#define DISPLAY_ALLON 				((uint8_t)0xA5)
#define DISPLAY_ON_NORMAL			((uint8_t)0xA4)	
#define DISPLAY_OFF						((uint8_t)0xAE)
#define DISPLAY_CHARGE_EN0		((uint8_t)0x8D)
#define DISPLAY_CHARGE_EN1		((uint8_t)0x14)
#define DISPLAY_WRITE					((uint16_t)0x0000)
#define DISPLAY_READ					((uint16_t)0x0001)
#define DISPLAY_WRITE_COMMAND	((uint8_t)0x00)
#define DISPLAY_WRITE_DATA		((uint8_t)0x40)
#define DISPLAY_WRITE_MULTIPLE ((uint8_t)0x80)
#define DISPLAY_SET_CLOCK     ((uint8_t)0xD5)
#define DISPLAY_SET_DIVIDE    ((uint8_t)0x80)
#define DISPLAY_SET_MULTIPLEX  ((uint8_t)0xA8)
#define DISPLAY_SET_MULTIPLEX2   ((uint8_t)0x3F)
#define DISPLAY_PRECHARGE_PERIOD ((uint8_t)0xD9)
#define DISPLAY_PRECHARGE_PERIOD2 ((uint8_t)0xF1)
// Number of bytes in startup sequence
#define STARTUP_LENGTH				((uint16_t)0x001E)
#define SET_COLADD_START0			((uint8_t)0x00)
#define SET_COLADD_START1			((uint8_t)0x10)
#define PAGE_MEMADD_MODE			((uint8_t)0x02)
#define HORIZ_MEMADD_MODE			((uint8_t)0x00)
#define SET_MEMADD_MODE				((uint8_t)0x20)
#define SET_COLADD						((uint8_t)0x21)
#define SET_PAGEADD						((uint8_t)0xAF)
#define SET_PAGEADD_START			((uint8_t)0xB0)
#define LOW_NIBBLE						((uint8_t)0x0F)
#define HIGH_NIBBLE						((uint8_t)0xF0)
// Constants for visualizing data on display
#define HR_X_START						((uint8_t)0x20)
#define HR_Y_START						((uint8_t)0x00)
#define DISPLAY_NORMAL				((uint8_t)0xA6)
#define DISPLAY_SCROLL_OFF		((uint8_t)0x2E)
#define DISPLAY_START_LINE		((uint8_t)0x40)
#define DISPLAY_COM_DIR   		((uint8_t)0xC8)
#define DISPLAY_COM_NORMAL		((uint8_t)0xC0)
#define DISPLAY_SET_OFFSET		((uint8_t)0xD3)
#define DISPLAY_SET_OFFSET1		((uint8_t)0x00)

extern Queue *displayQueue;
void display_Communicate(uint16_t numBytes, uint16_t readWrite, uint16_t dataCommand);
void display_Init(void);
void display_Int(uint16_t value);
extern void I2C2_EV_IRQHandler(void);

static const uint8_t FONT_BITMAP[43*12] = {
    0,124,198,206,222,246,230,198,198,124,0,0                    // '0' 48
    ,0,24,120,24,24,24,24,24,24,126,0,0                           // '1' 49
    ,0,124,198,198,12,24,48,96,198,254,0,0                        // '2' 50
    ,0,124,198,6,6,60,6,6,198,124,0,0                             // '3' 51
    ,0,12,28,60,108,204,254,12,12,12,0,0                          // '4' 52
    ,0,254,192,192,192,252,6,6,198,124,0,0                        // '5' 53
    ,0,124,198,192,192,252,198,198,198,124,0,0                    // '6' 54
    ,0,254,198,12,24,48,48,48,48,48,0,0                           // '7' 55
    ,0,124,198,198,198,124,198,198,198,124,0,0                    // '8' 56
    ,0,124,198,198,198,126,6,6,198,124,0,0                        // '9' 57
    ,0,0,0,12,12,0,0,12,12,0,0,0                                  // ':' 58
    ,0,0,0,12,12,0,0,12,12,12,24,0                                // '//' 59
    ,0,12,24,48,96,192,96,48,24,12,0,0                            // '<' 60
    ,0,0,0,0,254,0,254,0,0,0,0,0                                  // '=' 61
    ,0,96,48,24,12,6,12,24,48,96,0,0                              // '>' 62
    ,0,124,198,198,12,24,24,0,24,24,0,0                           // '?' 63
    ,0,124,198,198,222,222,222,220,192,126,0,0                    // '@' 64
    ,0,56,108,198,198,198,254,198,198,198,0,0                     // 'A' 65
    ,0,252,102,102,102,124,102,102,102,252,0,0                    // 'B' 66
    ,0,60,102,192,192,192,192,192,102,60,0,0                      // 'C' 67
    ,0,248,108,102,102,102,102,102,108,248,0,0                    // 'D' 68
    ,0,254,102,96,96,124,96,96,102,254,0,0                        // 'E' 69
    ,0,254,102,96,96,124,96,96,96,240,0,0                         // 'F' 70
    ,0,124,198,198,192,192,206,198,198,124,0,0                    // 'G' 71
    ,0,198,198,198,198,254,198,198,198,198,0,0                    // 'H' 72
    ,0,60,24,24,24,24,24,24,24,60,0,0                             // 'I' 73
    ,0,60,24,24,24,24,24,216,216,112,0,0                          // 'J' 74
    ,0,198,204,216,240,240,216,204,198,198,0,0                    // 'K' 75
    ,0,240,96,96,96,96,96,98,102,254,0,0                          // 'L' 76
    ,0,198,198,238,254,214,214,214,198,198,0,0                    // 'M' 77
    ,0,198,198,230,230,246,222,206,206,198,0,0                    // 'N' 78
    ,0,124,198,198,198,198,198,198,198,124,0,0                    // 'O' 79
    ,0,252,102,102,102,124,96,96,96,240,0,0                       // 'P' 80
    ,0,124,198,198,198,198,198,198,214,124,6,0                    // 'Q' 81
    ,0,252,102,102,102,124,120,108,102,230,0,0                    // 'R' 82
    ,0,124,198,192,96,56,12,6,198,124,0,0                         // 'S' 83
    ,0,126,90,24,24,24,24,24,24,60,0,0                            // 'T' 84
    ,0,198,198,198,198,198,198,198,198,124,0,0                    // 'U' 85
    ,0,198,198,198,198,198,198,108,56,16,0,0                      // 'V' 86
    ,0,198,198,214,214,214,254,238,198,198,0,0                    // 'W' 87
    ,0,198,198,108,56,56,56,108,198,198,0,0                       // 'X' 88
    ,0,102,102,102,102,60,24,24,24,60,0,0                         // 'Y' 89
    ,0,254,198,140,24,48,96,194,198,254,0,0};                       // 'Z' 90