
/*
  Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  version 2 as published by the Free Software Foundation.

  Added Arduino Due support from https://github.com/mcrosson/
*/

#ifndef __RF24_CONFIG_H__
#define __RF24_CONFIG_H__

#include <stddef.h>

/*** USER DEFINES:  ***/  
// #define FAILURE_HANDLING
// #define SERIAL_DEBUG  
/* #define MINIMAL_NO_FULL_API */
/**********************/
/* #include <stdlib.h> */
/* #include <stdint.h> */
/* #include <stdio.h> */
#include <string.h>

/* XTENSA Stuff */
#if defined(__XTENSA__)

/* Used with AVR */
#define PROGMEM 

/* Could have used BIT(x) macro */
#ifndef _BV
#define _BV(x) (1 << (x))
#endif

#define pgm_read_byte(p) (*(unsigned char*)(p))

/* Dont really need this, used for PROGMEM'ed 16 bit address values */
#define pgm_read_word(p) (*(unsigned short*)(p))

#define printf_P os_printf
#define printf os_printf
#define strlen_P strlen
#define PSTR(x) (x)
#define PRIPSTR "%s"

#endif /* __XTENSA__ */

// #define IF_SERIAL_DEBUG(x) ({x;})
#define IF_SERIAL_DEBUG(x)

#endif // __RF24_CONFIG_H__

