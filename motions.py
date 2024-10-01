# Imports
import rclpy

from rclpy.node import Node

from utilities import Logger, euler_from_quaternion
from rclpy.qos import QoSProfile

# TODO Part 3: Import message types needed: 
    # For sending velocity commands to the robot: Twist
    # For the sensors: Imu, LaserScan, and Odometry
# Check the online documentation to fill in the lines below
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from rclpy.time import Time

# You may add any other imports you may need/want to use below
# import ...


CIRCLE=0; SPIRAL=1; ACC_LINE=2
motion_types=['circle', 'spiral', 'line']

class motion_executioner(Node):
    
    def __init__(self, motion_type=0):
        
        super().__init__("motion_types")
        
        self.type=motion_type
        
        self.radius_=0.0
        
        self.successful_init=False
        self.imu_initialized=False
        self.odom_initialized=False
        self.laser_initialized=False
        
        # TODO Part 3: Create a publisher to send velocity commands by setting the proper parameters in (...)
        self.vel_publisher=self.create_publisher(Twist,'/cmd_vel',10)
                
        # loggers
        self.imu_logger=Logger('imu_content_'+str(motion_types[motion_type])+'.csv', headers=["acc_x", "acc_y", "angular_z", "stamp"])
        self.odom_logger=Logger('odom_content_'+str(motion_types[motion_type])+'.csv', headers=["x","y","th", "stamp"])
        self.laser_logger=Logger('laser_content_'+str(motion_types[motion_type])+'.csv', headers=["ranges", "angle_increment", "stamp"])
        
        # TODO Part 3: Create the QoS profile by setting the proper parameters in (...)
        qos=QoSProfile(reliability=2, durability=2, history=1, depth=10)

        # TODO Part 5: Create below the subscription to the topics corresponding to the respective sensors
        # IMU subscription
        
        self.subscription=self.create_subscription(Imu, "/imu", self.sub_callback,10)
        
        # ENCODER subscription

        self.subscription=self.create_subscription(Odometry, "/odom", self.sub_callback,10)
        
        # LaserScan subscription 
        
        self.subscription=self.create_subscription(LaserScan, "/laser", self.sub_callback,10)
        
        self.create_timer(0.1, self.timer_callback)


    # TODO Part 5: Callback functions: complete the callback functions of the three sensors to log the proper data.
    # To also log the time you need to use the rclpy Time class, each ros msg will come with a header, and then
    # inside the header you have a stamp that has the time in seconds and nanoseconds, you should log it in nanoseconds as 
    # such: Time.from_msg(imu_msg.header.stamp).nanoseconds
    # You can save the needed fields into a list, and pass the list to the log_values function in utilities.py

    def imu_callback(self, imu_msg: Imu):
        timestamp = Time.from_msg(imu_msg.header.stamp).nanoseconds
        
        imu_pos_x = imu_msg.orientation.x
        imu_pos_y = imu_msg.orientation.y
        imu_pos_z = imu_msg.orientation.z

        imu_av_x = imu_msg.angular_velocity.x
        imu_av_y = imu_msg.angular_velocity.y
        imu_av_z = imu_msg.angular_velocity.z

        imu_lacc_x = imu_msg.linear_acceleration.x 
        imu_lacc_y = imu_msg.linear_acceleration.y
        imu_lacc_z = imu_msg.linear_acceleration.z
        
        Logger.log_values([timestamp, imu_pos_x, imu_pos_y, imu_pos_z, imu_av_x, imu_av_y, imu_av_z, imu_lacc_x, imu_lacc_y, imu_lacc_z])
        
    def odom_callback(self, odom_msg: Odometry):
        timestamp = Time.from_msg(odom_msg.header.stamp).nanoseconds
        odom_orientation = odom_msg.pose.pose.orientation
        odom_x_pos = odom_msg.pose.pose.position.x
        odom_y_pos = odom_msg.pose.pose.position.y
        odom_z_pos = odom_msg.pose.pose.position.z
        odom_linear_vel = odom_msg.twist.twist.linear.x
        odom_angular_vel = odom_msg.twist.twist.angular.x
        
        Logger.log_values([timestamp, odom_orientation, odom_x_pos, odom_y_pos, odom_z_pos, odom_linear_vel, odom_angular_vel])
                
    def laser_callback(self, laser_msg: LaserScan):
        timestamp = Time.from_msg(laser_msg.header.stamp).nanoseconds
        
        laser_angle_min = laser_msg.angle_min
        laser_angle_max = laser_msg.angle_max
        laser_range_min = laser_msg.range_min
        laser_range_max = laser_msg.range_max
        
        Logger.log_values([timestamp, laser_angle_min, laser_angle_max, laser_range_min, laser_range_max])
                
    def timer_callback(self):
        
        if self.odom_initialized and self.laser_initialized and self.imu_initialized:
            self.successful_init=True
            
        if not self.successful_init:
            return
        
        cmd_vel_msg=Twist()
        
        if self.type==CIRCLE:
            cmd_vel_msg=self.make_circular_twist()
        
        elif self.type==SPIRAL:
            cmd_vel_msg=self.make_spiral_twist()
                        
        elif self.type==ACC_LINE:
            cmd_vel_msg=self.make_acc_line_twist()
            
        else:
            print("type not set successfully, 0: CIRCLE 1: SPIRAL and 2: ACCELERATED LINE")
            raise SystemExit 

        self.vel_publisher.publish(cmd_vel_msg)
        
    
    # TODO Part 4: Motion functions: complete the functions to generate the proper messages corresponding to the desired motions of the robot

    def make_circular_twist(self):
        
        msg=Twist()
        msg.linear.x = 0.2
        msg.angular.z = 0.5
        ... # fill up the twist msg for circular motion
        return msg

    def make_spiral_twist(self):
        msg=Twist()

        msg.linear.x = 0.2
        msg.angular.z = 0.5
        for i in range(30):
            msg.linear.x = msg.linear.x + 0.01
        ... # fill up the twist msg for spiral motion
        return msg
    
    def make_acc_line_twist(self):
        msg=Twist()
        msg.linear.x = 0.2
        msg.angular.z = 0
        ... # fill up the twist msg for line motion
        return msg

import argparse

if __name__=="__main__":
    

    argParser=argparse.ArgumentParser(description="input the motion type")


    argParser.add_argument("--motion", type=str, default="circle")



    rclpy.init()

    args = argParser.parse_args()

    if args.motion.lower() == "circle":

        ME=motion_executioner(motion_type=CIRCLE)
    elif args.motion.lower() == "line":
        ME=motion_executioner(motion_type=ACC_LINE)

    elif args.motion.lower() =="spiral":
        ME=motion_executioner(motion_type=SPIRAL)

    else:
        print(f"we don't have {arg.motion.lower()} motion type")


    
    try:
        rclpy.spin(ME)
    except KeyboardInterrupt:
        print("Exiting")
