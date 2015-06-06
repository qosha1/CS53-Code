#ifndef __bt_types__
#define __bt_types__

#include "stdint.h"

typedef struct _tagLAP_t
{
	uint8_t 	octet_1;
	uint8_t		octet_2;
	uint8_t 	octet_3;
} LAP_t;

typedef struct _tagBD_ADDR_t
{
	uint16_t	word_1;
	uint16_t	word_2;
	uint16_t	word_3;
} BD_ADDR_t;

typedef struct _tagLink_Key_t
{
	uint32_t	dword_1;
	uint32_t	dword_2;
	uint32_t	dword_3;
	uint32_t	dword_4;
} Link_Key_t;

typedef struct _tagClass_of_Device_t
{
	uint8_t 	octet_1;
	uint8_t 	octet_2;
	uint8_t 	octet_3;
} Class_of_Device_t;

#endif // __bt_types__