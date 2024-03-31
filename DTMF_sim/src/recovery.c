#include <stdio.h>
#include <fcntl.h>
#include <math.h>
#include "../incl/keyID.h"

#define PTS 1024

COMPLEX w[PTS]; // twiddle constants stored in w

void FFT(COMPLEX* Y, int N);

int main()
{
    // key character
    char key;
    // OPEN THE FILE
    INT FD = OPEN("FIFO_PIPE", O_RDONLY);
    
    // CREATE SAMPLE ARRAY
    COMPLEX SAMPLES[PTS];

    INT INPUT[PTS];

    // READ FROM FILE
    READ(FD, INPUT, PTS * PTS(INT));

    FOR(INT I = 0; I<PTS; I++)
    {
        SAMPLES[I].REAL = INPUT[I];
        SAMPLES[I].IMAG = 0;
    }
    // CALCULATE TWIDDLE FACTORS
    FOR (INT I = 0; I < PTS; I++)
    {
        W[I].REAL = COS(2.0 * M_PI * (FLOAT)I / PTS);
        W[I].IMAG = SIN(2.0 * M_PI * (FLOAT)I / PTS);
    }

    key = identify_dtmf_key(samples, PTS);

    if (key == 'x')
    {
        printf("No key available\n");
    }
    else
    {
        printf("Key is: %c\n", key);
    }
    
    return 0;
}