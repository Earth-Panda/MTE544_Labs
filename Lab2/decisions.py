# Imports


import sys

from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error
from pid import PID_ctrl

from rclpy import init, spin, spin_once
from rclpy.node import Node
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from localization import localization, rawSensor

from planner import TRAJECTORY_PLANNER, POINT_PLANNER, planner
from controller import controller, trajectoryController

# You may add any other imports you may need/want to use below
# import ...

##MAKE SURE TO RESET ODOM FRAME EACH RUN!!!

GOAL_THRESH_METRES = 0.01 # 1cm, can change if needed
#relax thresh if jittering too much near goal

class decision_maker(Node):
    
    def __init__(self, publisher_msg, publishing_topic, qos_publisher, goalPoint, rate=10, motion_type=POINT_PLANNER):

        super().__init__("decision_maker")

        #TODO Part 4: Create a publisher for the topic responsible for robot's motion
        self.publisher=self.create_publisher(publisher_msg, publishing_topic, qos_publisher)

        publishing_period=1/rate
        
        # Instantiate the controller
        # TODO Part 5: Tune your parameters here
    
        if motion_type == POINT_PLANNER:
            #P is good
            self.controller=controller(klp=1.2, klv=0.3, kli=0.05, kap=1.2, kav=0.5, kai=0.6)
            self.planner=planner(POINT_PLANNER)    
    
    
        elif motion_type==TRAJECTORY_PLANNER:
            self.controller=trajectoryController(klp=1.2, klv=0.3, kli=0.2, kap=1.2, kav=0.35, kai=0.2)
            self.planner=planner(TRAJECTORY_PLANNER)

        else:
            print("Error! you don't have this planner", file=sys.stderr)


        # Instantiate the localization, use rawSensor for now - rawSensor values are raw encoder values from odom
        self.localizer=localization(rawSensor) # creates localization node
        #main node is decision node, and localization node is node we call each time we need to get position

        # Instantiate the planner
        # NOTE: goalPoint is used only for the pointPlanner
        self.goal=self.planner.plan(goalPoint)

        self.create_timer(publishing_period, self.timerCallback)


    def timerCallback(self):
        
        # TODO Part 3: Run the localization node
        spin_once(self.localizer)   # Remember that this file is already running the decision_maker node.
    
        if self.localizer.getPose()  is  None:
            print("waiting for odom msgs ....")
            return

        vel_msg=Twist()

        curr_pose = self.localizer.getPose()
        print(f"curr pose -> x: {curr_pose[0]}, y: {curr_pose[1]}, theta: {curr_pose[2]}")
        
        # TODO Part 3: Check if you reached the goal
        if type(self.goal) == list:
            #this is for traj - bc traj is list - so therefore check if we are at last point in traj
            print(f"goal: {self.goal[-1][0]}, {self.goal[-1][1]}")
            reached_goal = calculate_linear_error(self.localizer.getPose(), self.goal[-1]) < GOAL_THRESH_METRES
        else: 
            print(f"goal pose -> x: {self.goal[0]}, y: {self.goal[1]}")
            #dont need to consider angular error bc we are not controlling it (i.e. it is not in goal pose)
            reached_goal = calculate_linear_error(self.localizer.getPose(), self.goal) < GOAL_THRESH_METRES
            #print(f"reached goal? {reached_goal}")
        

        if reached_goal:
            print("reached goal")
            self.publisher.publish(vel_msg) #empty (zeroed) commands bc want robot to stop moving
            
            self.controller.PID_angular.logger.save_log()
            self.controller.PID_linear.logger.save_log()
            
            #TODO Part 3: exit the spin
            raise SystemExit
        
        velocity, yaw_rate = self.controller.vel_request(self.localizer.getPose(), self.goal, True)
        print(f"lin velocity: {velocity}")
        print(f"ang velocity: {yaw_rate}")

        #TODO Part 4: Publish the velocity to move the robot
        vel_msg.linear.x = velocity
        vel_msg.angular.z = yaw_rate
        self.publisher.publish(vel_msg)

import argparse


def main(args=None):
    
    init()

    # TODO Part 3: You migh need to change the QoS profile based on whether you're using the real robot or in simulation.
    # Remember to define your QoS profile based on the information available in "ros2 topic info /odom --verbose" as explained in Tutorial 3
    odom_qos=QoSProfile(reliability=1, durability=1, history=1, depth=10)
    #Qs: for sim, rel=1, dur=1 (odd bc ros2 topic info /cmd_vel --verbose says rel=2 dur=1)
    #vel_qos = QoSProfile(reliability=2, durability=1, history=1, depth=10)
    #for turtlebots, use reliability=2, durability=2, history=1, depth=10
    #for simultion, run ros2 topic info /odom --verbose to see configs
    
    #change goal point in lab
    goalPoint = [-1.0, -1.0]
    # TODO Part 4: instantiate the decision_maker with the proper parameters for moving the robot
    #Qs: what is decision maker params? - I assume twist - but when why odom_qos passed in - its ok bc at least for sim, they have same qos - but then why we call if odom_qos, why not vel_qos
    if args.motion.lower() == "point":
        DM=decision_maker(Twist, "/cmd_vel", 10, goalPoint, 10, POINT_PLANNER)
    elif args.motion.lower() == "trajectory":
        DM=decision_maker(Twist, "/cmd_vel", 10, goalPoint, 10, TRAJECTORY_PLANNER)
    else:
        print("invalid motion type", file=sys.stderr)        
    
    try:
        spin(DM)
    except SystemExit:
        print(f"reached there successfully {DM.localizer.pose}")


if __name__=="__main__":

    argParser=argparse.ArgumentParser(description="point or trajectory") 
    argParser.add_argument("--motion", type=str, default="point")
    args = argParser.parse_args()

    main(args)
