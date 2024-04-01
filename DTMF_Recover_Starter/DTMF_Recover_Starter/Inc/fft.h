/** @file module.h
 * 
 * @brief A description of the module’s purpose. 
 *
 * @par       
 * COPYRIGHT NOTICE: (c) 2018 Barr Group.  All rights reserved.
 */ 

#ifndef FFT_H

#define FFT_H

#define PTS 1024		    //# of points for FFT

typedef struct {float real,imag;} COMPLEX;

extern COMPLEX w[PTS];       	    //twiddle constants stored in w

void FFT(COMPLEX *Y, int N);

#endif /* MODULE_H */

/*** end of file ***/