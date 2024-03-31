#include <stdio.h>
#include <fcntl.h>
#include <math.h>
#include "keyID.h"

#define PTS 1024
typedef struct
{
    float real, imag;
} COMPLEX;

COMPLEX w[PTS]; // twiddle constants stored in w

void FFT(COMPLEX* Y, int N);

int main()
{
    // key character
    char key;
    // open the file
    int fd = open("fifo_pipe", O_RDONLY);
    
    // create sample array
    COMPLEX samples[PTS];

    int input[PTS];

    // read from file
    read(fd, input, PTS * PTSof(int));

    for(int i = 0; i<PTS; i++)
    {
        samples[i].real = input[i];
        samples[i].imag = 0;
    }
    // calculate twiddle factors
    for (int i = 0; i < PTS; i++)
    {
        w[i].real = cos(2.0 * M_PI * (float)i / PTS);
        w[i].imag = sin(2.0 * M_PI * (float)i / PTS);
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