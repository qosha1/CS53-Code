/*****< buttons.h >********************************************************/
/*      California Institute of Technology 									  */
/*		EECS53 project 2015   		                          			      */
/*      Portable fitness  tracker                                             */
/*                                                                            */
/*  buttons.h   -       This little file helps out the touch sensing file     */
/*                                                                            */
/*  Author:  Quinn Osha                                                       */
/*                                                                            */
/*** MODIFICATION HISTORY *****************************************************/
/*                                                                            */
/*   mm/dd/yy  F. Lastname    Description of Modification                     */
/*   --------  -----------    ------------------------------------------------*/
/*   05/25/15  Q. Osha        Initial revision.                               */
/******************************************************************************/

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

#define CAP_BUTTON1_MASK		0x01    /* Bit positions of returned button press
#define CAP_BUTTON2_MASK		0x02     * values */
#define CAP_BUTTON3_MASK		0x04
#define CAP_BUTTON4_MASK		0x08

/* Function declarations */
void buttons_Init();
uint8_t get_Button();