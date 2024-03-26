#include <stdio.h>
#include <fcntl.h>
#include <math.h>

#define PTS 1024
#define PTS 1024 // # of points for FFT
typedef struct
{
    float real, imag;
} COMPLEX;
COMPLEX w[PTS]; // twiddle constants stored in w
void FFT(COMPLEX* Y, int N);

int main()
{
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

    FFT(samples, PTS);

    int output[PTS];

    for (int i = 0; i< PTS; i++)
    {
        output[i] = sqrt(((int)(samples[i].real))^2 + ((int)(samples[i].imag))^2);
    }
    int fs = open("spectrum_data", O_CREAT | O_WRONLY | O_BINARY);

    write(fs, output, PTS * PTSof(int));
    close(fs);
    close(fd);
    return 0;
}



void FFT(COMPLEX *Y, int N) // input sample array, # of points
{
    COMPLEX temp1, temp2;     // temporary storage variables
    int i, j, k;              // loop counter variables
    int upper_leg, lower_leg; // index of upper/lower butterfly leg
    int leg_diff;             // difference between upper/lower leg
    int num_stages = 0;       // number of FFT stages (iterations)
    int index, step;          // index/step through twiddle constant
    i = 1;                    // log(base2) of N points= # of stages
    do
    {
        num_stages += 1;
        i = i * 2;
    } while (i != N);
    leg_diff = N / 2;                // difference between upper&lower legs
    step = PTS / N;                  // step between values in twiddle.h
    for (i = 0; i < num_stages; i++) // for N-point FFT
    {
        index = 0;
        for (j = 0; j < leg_diff; j++)
        {
            for (upper_leg = j; upper_leg < N; upper_leg += (2 * leg_diff))
            {
                lower_leg = upper_leg + leg_diff;
                temp1.real = (Y[upper_leg]).real + (Y[lower_leg]).real;
                temp1.imag = (Y[upper_leg]).imag + (Y[lower_leg]).imag;
                temp2.real = (Y[upper_leg]).real - (Y[lower_leg]).real;
                temp2.imag = (Y[upper_leg]).imag - (Y[lower_leg]).imag;
                (Y[lower_leg]).real = temp2.real * (w[index]).real - temp2.imag * (w[index]).imag;
                (Y[lower_leg]).imag = temp2.real * (w[index]).imag + temp2.imag * (w[index]).real;
                (Y[upper_leg]).real = temp1.real;
                (Y[upper_leg]).imag = temp1.imag;
            }
            index += step;
        }
        leg_diff = leg_diff / 2;
        step *= 2;
    }
    j = 0;
    for (i = 1; i < (N - 1); i++) // bit reversal for resequencing data
    {
        k = N / 2;
        while (k <= j)
        {
            j = j - k;
            k = k / 2;
        }
        j = j + k;
        if (i < j)
        {
            temp1.real = (Y[j]).real;
            temp1.imag = (Y[j]).imag;
            (Y[j]).real = (Y[i]).real;
            (Y[j]).imag = (Y[i]).imag;
            (Y[i]).real = temp1.real;
            (Y[i]).imag = temp1.imag;
        }
    }
    return;
}

int* find_peaks(int* spectrum)
{
    // create an edge detection kernel
    int kernel[3] = {-1, 0, 1};

    // create a peaks array
    int value;
    int edges[PTS];
    int peaks[10];

    // sum variable for average
    int sum = 0;

    for (int i = 2; i < PTS; i++)
    {
        // find edges
        value = 
            spectrum[i]*kernel[2] +
            spectrum[i-1]*kernel[1] +
            spectrum[i-2]*kernel[0];

        // add
        edges[i-2]=value;
        sum += value;
    }

    // find peaks larger than average that are separated by more than 2 spots
    int average = sum/(PTS);
    int index = 0;
    int close_peaks=0;
    for (int i = 0; i < PTS; i++)
    {
        if((edges[i] > (average + 30)) & close_peaks == 0)
        {
            peaks[index] = i;
            index++;
            close_peaks = 2;
        }
        else
        {
            close_peaks <= 0 ? close_peaks = 0 : close_peaks--;
        }
    }

    return peaks;
}
