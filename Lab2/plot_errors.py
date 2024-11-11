import matplotlib.pyplot as plt
from utilities import FileReader




def plot_errors(filename):
    
    # Reading headers and values from the file
    headers, values = FileReader(filename).read_file()
    
    # Extracting the time values and normalizing to the first timestamp
    time_list = []
    first_stamp = values[0][-1]
    
    for val in values:
        time_list.append(val[-1] - first_stamp)

    # Creating subplots
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    # State Space Plot
    axes[0].plot([lin[0] for lin in values], 
                [lin[1] for lin in values], marker='o')
    axes[0].set_title("PID Controller: Linear Error vs. Linear Error Derivative")
    axes[0].set_xlabel("Linear Error (m)")  # Label for x-axis
    axes[0].set_ylabel("Linear Error Derivative (m/s)")  # Label for y-axis
    axes[0].legend()
    axes[0].grid(True)
    
    # Error State Behaviour Plot
    axes[1].set_title("PD Controller (Parabola): Angular Error State Behaviour")
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("Error Value")
    
    for i in range(0, len(headers) - 1):
        axes[1].plot(time_list, 
                     [lin[i] for lin in values], 
                     label=headers[i] + " angular", marker='o')

    axes[1].legend()
    axes[1].grid(True)
    plt.tight_layout()
    plt.show()
    
    





import argparse

if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    
    args = parser.parse_args()
    
    print("plotting the files", args.files)

    filenames=args.files
    for filename in filenames:
        plot_errors(filename)



