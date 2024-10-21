import numpy as np


from pid import PID_ctrl
from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error

M_PI=3.1415926535

P=0; PD=1; PI=2; PID=3

MAX_TRANS_VEL_SIM = 0.22
MAX_ROT_VEL_SIM = 2.84
MAX_TRANS_VEL_REAL = 0.31 
MAX_ROT_VEL_REAL = 1.9

class controller:
    
    
    # Default gains of the controller for linear and angular motions
    def __init__(self, klp=0.2, klv=0.2, kli=0.2, kap=0.2, kav=0.2, kai=0.2):
        
        # TODO Part 5 and 6: Modify the below lines to test your PD, PI, and PID controller
        self.PID_linear=PID_ctrl(P, klp, klv, kli, filename_="linear.csv")
        self.PID_angular=PID_ctrl(P, kap, kav, kai, filename_="angular.csv")

    
    def vel_request(self, pose, goal, status):
        
        e_lin=calculate_linear_error(pose, goal)
        e_ang=calculate_angular_error(pose, goal)

        print(f"lin error: {e_lin}")
        print(f"ang error: {e_ang}")
        linear_vel=self.PID_linear.update([e_lin, pose[3]], status)
        angular_vel=self.PID_angular.update([e_ang, pose[3]], status)
        # TODO Part 4: Add saturation limits for the robot linear and angular velocity

        #Qs: why were the limits in the if condn set to 1? shouldnt they be the max allowable value?
        #also shouldnt be account for negative values?
        if linear_vel > MAX_TRANS_VEL_SIM:
            linear_vel = MAX_TRANS_VEL_SIM
        elif linear_vel < -MAX_TRANS_VEL_SIM:
            linear_vel = -MAX_TRANS_VEL_SIM

        if angular_vel > MAX_ROT_VEL_SIM:
            angular_vel = MAX_ROT_VEL_SIM
        elif angular_vel < -MAX_ROT_VEL_SIM:
            angular_vel = -MAX_ROT_VEL_SIM

        #linear_vel =  MAX_TRANS_VEL_SIM if linear_vel > 1.0 else linear_vel
        #angular_vel = MAX_ROT_VEL_SIM if angular_vel > 1.0 else angular_vel
        
        return linear_vel, angular_vel
    

class trajectoryController(controller):

    def __init__(self, klp=0.2, klv=0.2, kli=0.2, kap=0.2, kav=0.2, kai=0.2):
        
        super().__init__(klp, klv, kli, kap, kav, kai)
    
    def vel_request(self, pose, listGoals, status):
        
        goal=self.lookFarFor(pose, listGoals)
        
        finalGoal=listGoals[-1]
        
        e_lin=calculate_linear_error(pose, finalGoal)
        e_ang=calculate_angular_error(pose, goal)

        
        linear_vel=self.PID_linear.update([e_lin, pose[3]], status)
        angular_vel=self.PID_angular.update([e_ang, pose[3]], status) 

        # TODO Part 5: Add saturation limits for the robot linear and angular velocity

        linear_vel = ... if linear_vel > ... else linear_vel
        angular_vel= ... if angular_vel > ... else angular_vel
        
        return linear_vel, angular_vel

    def lookFarFor(self, pose, listGoals):
        
        poseArray=np.array([pose[0], pose[1]]) 
        listGoalsArray=np.array(listGoals)

        distanceSquared=np.sum((listGoalsArray-poseArray)**2,
                               axis=1)
        closestIndex=np.argmin(distanceSquared)

        return listGoals[ min(closestIndex + 3, len(listGoals) - 1) ]
