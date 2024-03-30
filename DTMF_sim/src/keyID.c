#include "keyID.h"

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

	// convert to magnitude spectrum data
	for (int index = 0; index < N; index++)
	{
		spectrum[index] = sqrt((samples[point].real)^2+(samples[point].imag)^2);
		average += spectrum[index];
	}

	// caluclate average
	average = average / (float)N;

	

	return key;
}

