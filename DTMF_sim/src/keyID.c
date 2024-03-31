#include "keyID.h"

/*
* Takes an array of samples of size N and returns the DTMF key character.
* Requires fft.c and fft.h included in the project library.
* @param samples - the array of signal samples
* @param N - the number of samples in the array
* @return key - the character of the key pressed. If no key present it returns 'x'.
*/

char identify_dtmf_key(COMPLEX* samples, int N)
{
	// create key variable
	char key = '';

	// create spectrum data array
	float spectrum[N];

	// create average value for later
	float average = 0.0;

	// dtmf frequency array
	int dtmf_row = {697, 770, 852, 941};
	int dtmf_column = {1209, 1336, 1477, 1633};
	float bin_size = ((float)FS)/((float)N);

	// key array TODO
		// row and column index variables
	int row_index = 5;
	int column_index = 5;
		// key character array
	char keys[4][4] =
	{
		{'1','2','3','A'},
		{'4','5','6','B'},
		{'7','8','9','C'},
		{'*','0','#','D'}
	}
	// calculate key row indicies
	for (int row = 0; row < 4; row++)
	{
		dtmf_row = (int)(((float)dtmf_row[row])/bin_size + 0.5); // round to nearest integer
	}

	// calculate key column indices
	for (int column = 0; column < 4; column++)
	{
		dtmf_row = (int)(((float)dtmf_row[row])/bin_size + 0.5); // round to nearest integer
	}

	// calculate FFT
	FFT(samples, N);

	// convert to magnitude spectrum data and calculate the average sample value
	for (int index = 0; index < N; index++)
	{
		spectrum[index] = sqrt((samples[point].real)^2+(samples[point].imag)^2);
		average += spectrum[index];
	}

	// caluclate average
	average = average / (float)N;

	// calculate peak cutoff
	float cutoff = average*THRESHOLD;
	float row_peak = 0.0;
	float column_peak = 0.0;

	// check magnitude spectrum for peaks at key indexes

	for(int index = 0; index < 4; index++)
	{
		// check if a key has a peak larger than the cutoff AND larger than the current largest peak found
			// for the rows
		if((spectrum[dtmf_row[index]] > cutoff) & (spectrum[dtmf_row[index]] > row_peak)
		{
			row_peak = spectrum[dtmf_row[index]];
			row_index = index;
		}

			// for the columns
		if((spectrum[dtmf_column[index]] > cutoff) & (spectrum[dtmf_column[index]] > column_peak)
		{
			column_peak = spectrum[dtmf_column[index]];
			column_index = index;
		}
	}

	// if no key, return 'x', else index to the key array and return that key
	if (row_index > 4 | column_index > 4)
	{
		key = 'x';
	}
	else
	{
		key=keys[row_index][column_index];
	}
	return key;
}

