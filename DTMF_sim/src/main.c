#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <fcntl.h>

#define COLUMN 300
#define ROW 500
#define SIZE 500

typedef struct Node
{
	struct Node* prev;
	float value;
	struct Node* next;
}Node;

typedef struct DE
{
	float numerator[3];
	float denominator[3];
}DE;

// stuff for this program

Node* input_buffer;
Node* row_buffer;
Node* column_buffer;

DE column_DE;
DE row_DE;
// function prototypes
// creates a circular buffer of size 'size'
Node* create_circ_buffer(int size);
// gets the next sample for a given buffer from a difference equation
float next_sample(Node* buff, DE diff_eq);
//calculate filter coefficients for a frequency and returns a difference equation variable
DE calculate_coeff(int freq);

// Add your global variables here

int sample = 0;

// add your main loop code here
int main()
{
	int samples [SIZE];
	// your non-loop code here
	/* %%%%%%%%%%%%%%%%% compute filter coefficients %%%%%%%%%%%%%%%%%%%*/

	row_DE = calculate_coeff(ROW);
	column_DE = calculate_coeff(COLUMN);

	/* %%%%%%%%%%%%%%%%% Declare circular buffers %%%%%%%%%%%%%%%%%%%*/
	input_buffer = create_circ_buffer(3);
	row_buffer = create_circ_buffer(3);
	column_buffer = create_circ_buffer(3);

	input_buffer->value = 10000; // start the sine wave

	int fd = open("fifo_pipe", O_WRONLY | O_BINARY);
	for (int i = 0; i<SIZE; i++)
	{
		// put your load_buffer code here
		//move to next sample in buffer
		row_buffer = row_buffer->next;
		column_buffer = column_buffer->next;
		input_buffer = input_buffer->next;

		// calculate new output value
		row_buffer->value = next_sample(row_buffer, row_DE);
		column_buffer->value = next_sample(column_buffer, column_DE);

		// set input buffer value to 0 since its an impulse function
		input_buffer->value = 0;

		// cast to sample
		int sample1 = (int)(row_buffer->value + column_buffer->value);
		samples[i] = sample1;
	}
	write(fd, samples, SIZE * sizeof(int));
	close(fd);
	return 0;
}

// function defines
float next_sample(Node* output_buff, DE diff_eq)
{
	// create value variable to be returned, init to 0
	float value = 0;
	// create temporary variables to store buffer pointers
	Node* temp_input = input_buffer;
	Node* temp_output = output_buff;

	// iterate through DE and add stuff up
	for (int i = 0; i < 3; i++)
	{
		value += diff_eq.numerator[i]*(temp_input->value)+diff_eq.denominator[i]*(temp_output->value);
		
		// go to previous buffer slot
		temp_input = temp_input->prev;
		temp_output = temp_output->prev;
	}
	// return the next sample value
	return value;
}

Node* create_circ_buffer(int size)
{
	// create head node
	Node* head = malloc(sizeof(Node));

	// initialize values to 0
	head->value = 0;

	// declare a pointer to the current node
	Node* current;

	// set to head since currently head is the only node that exists
	current = head;

	//decrement size by 1
	size--;
	// create the rest of the nodes
	for (int i = 0; i < size; i++)
	{
		// create new node
		Node* new_node = malloc(sizeof(Node));
		new_node->value = 0;
		// point current to new_node and vice versa
		current->next = new_node;
		new_node->prev = current;
		// set current node to next node
		current = new_node;
	}
	// point head to tail and tail to head
	head->prev = current;
	current->next = head;
	//return the start of the circular buffer
	return head;
}

DE calculate_coeff(int freq)
{
	// create a difference equation object
	DE diff_eq;
	//calculate coefficients
	// numerator
	diff_eq.numerator[0] = 0;
	diff_eq.numerator[1] = sin((float)freq*M_PI/24000.0);
	diff_eq.numerator[2] = 0;
	//denominator
	diff_eq.denominator[0]= 0;
	diff_eq.denominator[1] = 2*cos((float)freq*M_PI/24000.0);
	diff_eq.denominator[2] = -1;

	return diff_eq;
}