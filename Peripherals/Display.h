/*****< display.h >********************************************************/
/*      California Institute of Technology 									  */
/*		EECS53 project 2015   		                          			      */
/*      Portable fitness  tracker                                             */
/*                                                                            */
/*  display.h   -       This header keeps track of the different command codes*/
/*                      and display specific constants.                       */
/*                                                                            */
/*  Author:  Quinn Osha                                                       */
/*                                                                            */
/*** MODIFICATION HISTORY *****************************************************/
/*                                                                            */
/*   mm/dd/yy  F. Lastname    Description of Modification                     */
/*   --------  -----------    ------------------------------------------------*/
/*   03/17/15  Q. Osha        Initial revision.                               */
/*   06/05/15  Q. Osha        Added command for horizontal addressing. 
/******************************************************************************/

#ifndef _display_
#define _display_

#include <stdint.h>
#include "stm32f373xc.h"                  // Device header
#include "Queue.h"
#include "../Main/mpu_constants.h"

#define I2C_TIMING_PRESC			((uint32_t)0x00000001)
#define I2C_TIMING_SCLDEL			((uint32_t)0x00000004)
#define I2C_TIMING_SDADEL			((uint32_t)0x00000002)
#define I2C_TIMING_SCLH				((uint32_t)0x0000000F)
#define I2C_TIMING_SCLL				((uint32_t)0x00000013)

// display event queue
#define DISPLAY_QUEUE_LENGTH	((uint32_t)0x00000100)
#define DISPLAY_RESET_TIME		((uint32_t)0x00000100)

/* Display characteristics */
#define DISPLAY_ADDRESS 			((uint32_t)0x00000078)
#define DISPLAY_DATA_ADDRESS 	((uint32_t)0x0000007A)
#define DISPLAY_WIDTH					0x0040
#define DISPLAY_HEIGHT				0x0006

/* Implementation specific data locations on display */
#define ACCELX_DISPLAY_PAGE		0
#define ACCELY_DISPLAY_PAGE		1
#define ACCELZ_DISPLAY_PAGE		2
#define GYROX_DISPLAY_PAGE		3
#define GYROY_DISPLAY_PAGE		4
#define GYROZ_DISPLAY_PAGE		5
#define MOTION_DATA_COLSTART	18
#define DATA_LABELS_COLSTART	0

/* Display command codes */
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
#define DISPLAY_SET_COM_PINS	((uint8_t)0xDA)
#define DISPLAY_SET_COM_PINS2	((uint8_t)0x02)

// Number of bytes in startup sequence
#define STARTUP_LENGTH				((uint16_t)0x001E)
#define SET_COLADD_START0			((uint8_t)0x00)
#define SET_COLADD_START1			((uint8_t)0x10)
#define PAGE_MEMADD_MODE			((uint8_t)0x02)
#define HORIZ_MEMADD_MODE			((uint8_t)0x00)
#define SET_MEMADD_MODE				((uint8_t)0x20)
#define SET_COLADD						((uint8_t)0x21)
#define SET_PAGE_ADDRESS			((uint8_t)0x22)
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
#define DISPLAY_SET_COM_SEQUENTIAL	((uint8_t)0x00)
#define DISPLAY_SET_COM_REMAP	((uint8_t)0x20)
#define DISPLAY_COLADD_START	((uint8_t)0x20)
#define DISPLAY_COLADD_END		((uint8_t)0x5F)
#define DISPLAY_PAGEADD_START	((uint8_t)0x00)
#define DISPLAY_PAGEADD_END		((uint8_t)0x05)

extern Queue *displayQueue;
void display_Communicate(uint16_t numBytes, uint16_t readWrite, uint16_t dataCommand);
void display_Init(void);
void display_Shutdown();
void display_Startup();
void display_Int(uint16_t value, uint8_t display_page, 
									uint8_t x_offset, boolean display_now);
void display_String(char *string, uint8_t display_page, 
									uint8_t x_offset, boolean display_now);
extern void I2C2_EV_IRQHandler(void);

#endif //_display