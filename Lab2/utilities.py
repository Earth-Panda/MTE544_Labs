from math import atan2, asin, sqrt

M_PI=3.1415926535

class Logger:
    
    def __init__(self, filename, headers=["e", "e_dot", "e_int", "stamp"]):
        
        self.filename = filename

        with open(self.filename, 'w') as file:
            
            header_str=""

            for header in headers:
                header_str+=header
                header_str+=", "
            
            header_str+="\n"
            
            file.write(header_str)


    def log_values(self, values_list):

        with open(self.filename, 'a') as file:
            
            vals_str=""
            
            for value in values_list:
                vals_str+=f"{value}, "
            
            vals_str+="\n"
            
            file.write(vals_str)
            

    def save_log(self):
        pass

class FileReader:
    def __init__(self, filename):
        
        self.filename = filename
        
        
    def read_file(self):
        
        read_headers=False

        table=[]
        headers=[]
        with open(self.filename, 'r') as file:

            if not read_headers:
                for line in file:
                    values=line.strip().split(',')

                    for val in values:
                        if val=='':
                            break
                        headers.append(val.strip())

                    read_headers=True
                    break
            
            next(file)
            
            # Read each line and extract values
            for line in file:
                values = line.strip().split(',')
                
                row=[]                
                
                for val in values:
                    if val=='':
                        break
                    row.append(float(val.strip()))

                table.append(row)
        
        return headers, table
    
    

# TODO Part 3: Implement the conversion from Quaternion to Euler Angles
def euler_from_quaternion(quat):
    """
    Convert quaternion (w in last place) to euler roll, pitch, yaw.
    quat = [x, y, z, w]
    """
    #roll (rotation about x-axis)
    sin_r = 2 * (quat.w * quat.x + quat.y * quat.z)
    cos_r = 1 - 2 * (quat.x * quat.x + quat.y * quat.y)
    roll = atan2(sin_r, cos_r)

    #pitch (rotation about y-axis)
    sin_p = sqrt(1 + 2 * (quat.w * quat.y - quat.x * quat.z))
    cos_p = sqrt(1 - 2 * (quat.w * quat.y - quat.x * quat.z))
    pitch = 2 * atan2(sin_p, cos_p) - M_PI / 2

    #yaw (rotation about z-axis)
    sin_y = 2 * (quat.w * quat.z + quat.x * quat.y)
    cos_y = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
    yaw = atan2(sin_y, cos_y)
    return yaw


#TODO Part 4: Implement the calculation of the linear error
def calculate_linear_error(current_pose, goal_pose):
        
    # Compute the linear error in x and y
    # Remember that current_pose = [x,y, theta, time stamp] and goal_pose = [x,y]
    # Remember to use the Euclidean distance to calculate the error.
    deltaY = goal_pose[1] - current_pose[1]
    deltaX = goal_pose[0] - current_pose[0]
    error_linear = sqrt(deltaX**2 + deltaY**2)

    return error_linear

#TODO Part 4: Implement the calculation of the angular error
def calculate_angular_error(current_pose, goal_pose):

    # Compute the linear error in x and y
    # Remember that current_pose = [x,y, theta, time stamp] and goal_pose = [x,y]
    # Use atan2 to find the desired orientation
    # Remember that this function returns the difference in orientation between where the robot currently faces and where it should face to reach the goal
    deltaY = goal_pose[1] - current_pose[1]
    deltaX = goal_pose[0] - current_pose[0]
    #print(f"atan2 {atan2(deltaY, deltaX)}")
    error_angular = atan2(deltaY, deltaX)-current_pose[2]

    # Remember to handle the cases where the angular error might exceed the range [-π, π] - Qs: when might this occur?? doesnt atan2 always return angle between -pi and pi
    if error_angular > M_PI :
        error_angular = error_angular - ((error_angular+M_PI)//(2*M_PI))*2*M_PI
    elif error_angular < -M_PI:
        error_angular = error_angular + ((abs(error_angular)+M_PI)//(2*M_PI))*2*M_PI
    
    return error_angular
