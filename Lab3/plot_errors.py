import matplotlib.pyplot as plt
from utilities import FileReader
import numpy as np




def plot_errors(filename):
    
    headers, values=FileReader(filename).read_file()
    

    time_list=[]
    
    first_stamp=values[0][-1]
    seconds_list = []
    for val in values:
        time_list.append(val[-1] - first_stamp)
        #seconds_list.append((val[-1] - first_stamp)*1e-9)

    dt = 0.1
    v = 0
    w = 1
    t = 0
    max_time = 20
    x = [0]
    y = [0]
    th = [0]
    while t < max_time:
        v += 0.01 if v < 1.0 else 0.0
        th.append(th[-1] + w * dt)
        x.append(x[-1] + v*np.cos(th[-1])*dt)
        y.append(y[-1] + v*np.sin(th[-1])*dt)
        t += dt
    
    fig, axes = plt.subplots(2,1, figsize=(14,6))

    # plot ground truth
    #axes[0].plot([lin[len(g_headers)-1] for lin in g_values], [lin[len(g_headers) - 1] for lin in g_values])
    axes[0].plot(x, y)
    # plot data 
    axes[0].plot([lin[len(headers) - 3] for lin in values], [lin[len(headers) - 2] for lin in values])

    axes[0].set_title("state space")
    axes[0].legend(["truth", "estimate"])
    axes[0].grid()

    
    axes[1].set_title("each individual state")
    for i in range(0, len(headers) - 1):
        axes[1].plot(time_list, [lin[i] for lin in values], label= headers[i])

    axes[1].legend()
    axes[1].grid()

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


