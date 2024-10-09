# You can use this file to plot the loged sensor data
# Note that you need to modify/adapt it to your own files
# Feel free to make any modifications/additions here

import matplotlib.pyplot as plt
from utilities import FileReader

def plot_imu(filename, type):
    
    headers, values=FileReader(filename).read_file() 
    time_list=[]
    first_stamp=values[0][-1]
    
    for val in values:
        time_list.append(val[-1] - first_stamp)

    plt.suptitle("IMU data for " + type)
    plt.subplot(1, 2, 1)
    for i in range(0, len(headers) - 1):
        if(i < len(headers) - 2):
            plt.plot(time_list, [lin[i] for lin in values], label= headers[i]+ " linear")
            plt.xlabel("time (s)")
            plt.ylabel("acceleration (m/s^2)")
            plt.legend()
        else:
            plt.subplot(1, 2, 2)
            plt.plot(time_list, [lin[i] for lin in values], label= headers[i]+ " linear")
            plt.xlabel("time (s)")
            plt.ylabel("angular velocity (rad/s)")
            plt.legend()
    
    plt.grid()
    plt.show()

def plot_odom(filename, type):
    
    headers, values=FileReader(filename).read_file() 
    time_list=[]
    first_stamp=values[0][-1]
    
    for val in values:
        time_list.append(val[-1] - first_stamp)

    plt.suptitle("Odom data for " + type)
    plt.subplot(1, 2, 1)
    for i in range(0, len(headers) - 1):
        if(i < len(headers) - 2):

            plt.plot(time_list, [lin[i] for lin in values], label= headers[i]+ " linear")
            plt.xlabel("time (s)")
            plt.ylabel("position (m)")
            plt.legend()
        else:
            plt.subplot(1, 2, 2)
            plt.plot(time_list, [lin[i] for lin in values], label= headers[i]+ " linear")
            plt.xlabel("time (s)")
            plt.ylabel("angle (rad)")
            plt.legend()
    
    plt.grid()
    plt.show()

import argparse

types = ["Spiral", "Line", "Circle"]

if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    
    args = parser.parse_args()
    
    print("plotting the files", args.files)

    filenames=args.files
    for i, filename in enumerate(filenames):
        plot_odom(filename, types[i])
