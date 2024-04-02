

/** @file module.h
 * 
 * @brief A description of the module’s purpose. 
 *
 * @par       
 * COPYRIGHT NOTICE: (c) 2018 Barr Group.  All rights reserved.
 */ 

#ifndef KEYID_H
#define KEYID_H

#include "fft.h"
#include <math.h>

// define sampling frequency
#define FS 48000
#define THRESHOLD 3.0

char identify_dtmf_key(COMPLEX* samples);

#endif /* KEYID_H */

/*** end of file ***/
