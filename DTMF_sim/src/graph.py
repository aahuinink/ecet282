import sys
import matplotlib.pyplot as plt
import numpy as np
import os

def plot_graph(values):
    plt.plot(values)
    plt.xlabel('Index')
    plt.ylabel('Value')
    plt.title('Graph of Values')
    plt.grid(True)
    plt.savefig('graph.png')  # Save the plot as an image
    plt.xlim((0,500))
    plt.show()

def main():
    with open("fifo_pipe", 'rb') as fifo:
        values = np.fromfile(fifo, dtype=int)


    plot_graph(values)
    os.unlink("fifo_pipe")

if __name__ == "__main__":
    main()
