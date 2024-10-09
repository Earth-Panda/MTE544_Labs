import matplotlib.pyplot as plt
import numpy as np
from math import sin, cos
from utilities import FileReader

def polarToCartesian(dist, angle):
    return dist*cos(angle), dist*sin(angle)

def rawLidarToCartesian(rangeValues, angleIncrement):
    #assume first value at 0 rad
    angle = 0
    pointList = []
    for val in rangeValues:
        if val != float("inf"):
            x, y = polarToCartesian(float(val), angle)
            pointList.append((x, y))
        angle+=angleIncrement
    return pointList

def extractLidarData(laser_ranges_file, laser_content_file):
    rangeHeaders, rangeValues = FileReader(laser_ranges_file).read_file()
    contantHeaders, contentValues = FileReader(laser_content_file).read_file()

    for values in rangeValues:
        #remove timestamp at end of each range list
        values.pop()

    angleIncrements = []
    for values in contentValues:
        #only want angle increments from content values, not timestamp
        angleIncrements.append(values[0])

    cartesianPointsTable = []

    for i, rangeSet in enumerate(rangeValues):
        angleIncrement = float(angleIncrements[i])
        cartesianPointsTable.append(rawLidarToCartesian(rangeValues[0], angleIncrement))
    return cartesianPointsTable

def main():
    #show first revolution of lidar points
    lidarPoints = extractLidarData("Lab1_results/csvs/laser_content_circle_ranges.csv", "Lab1_results/csvs/laser_content_circle.csv")
    x, y = zip(*lidarPoints[0])
    plt.scatter(x, y, c="green")
    plt.title("Laser Scan Data for One Revolution")
    plt.xlabel("x[m]")
    plt.ylabel("y[m]")
    plt.show()

    #show slam map
    map = plt.imread("Lab1_results/map.pgm")
    plt.imshow(map)
    plt.title("SLAM Top Down View of Lab Room")
    plt.show()
    
if __name__ == "__main__":
    main()