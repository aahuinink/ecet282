import sys
import matplotlib.pyplot as plt
import numpy as np
import os

file_name = "spectrum_data"

def plot_graph(values):
    plt.figure()
    plt.plot(values)
    plt.xlabel('Index')
    plt.ylabel('Value')
    plt.title('Graph of Values')
    plt.grid(True)
    plt.savefig('graph.png')  # Save the plot as an image
    plt.xlim((0,1024))
    plt.show()

def main():
    with open(file_name, 'rb') as fifo:
        values = np.fromfile(fifo, dtype=int)

    with open("fifo_pipe", 'rb') as wave:
        wave_vals = np.fromfile(wave, dtype=int)

    plot_graph(values)
    plot_graph(wave_vals)
    fifo.close()
    wave.close()

if __name__ == "__main__":
    main()
