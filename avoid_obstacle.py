#!/usr/bin/env python3  

from pyrsistent import v
import rospy
import numpy as np 
from sensor_msgs.msg import LaserScan   
from geometry_msgs.msg import Twist 

# This class receives a LaserScan and finds the closest object  
class AvoidObstacleClass():  
    def __init__(self):  

        rospy.on_shutdown(self.cleanup)  

        ############################### SUBSCRIBERS #####################################  
        rospy.Subscriber("base_scan", LaserScan, self.laser_cb)  

        ############################### PUBLISHERS ##################################### 
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1) 

        ############################### VARIABLES ##################################### 
        vel_msg = Twist() 
        self.closest_range = 0.0 #Distance to the closest object 
        self.closest_angle = 0.0 #Angle to the closest object 

        ############ CONSTANTS ################  
        # kv=0.2 #Constant to change the linear speed 
        kw=0.5 #Angular velocity gain 
        v = 0.5 # Linear velocity

        #********** INIT NODE **********###  
        Hz = 10
        r = rospy.Rate(Hz) #10Hz is the lidar's frequency  
        print("Node initialized (" + str(Hz) + "Hz)") 

        while not rospy.is_shutdown():  
            range=self.closest_range 
            theta_closest=self.closest_angle 

            #limit the angle to [-pi,pi] 
            theta_closest=np.arctan2(np.sin(theta_closest),np.cos(theta_closest)) 
            print("theta: " + str(theta_closest)) 

            if np.isposinf(range):  
                print("No object detected") 
                vel_msg.linear.x = v
                vel_msg.angular.z = 0.0 

            else: # If there is any obstacle
                theta_avoid_obst = np.pi - theta_closest
                vel_msg.linear.x = v

                if -np.pi/2 < theta_closest < np.pi/2:
                    vel_msg.angular.z = kw*theta_avoid_obst

                else:
                    vel_msg.angular.z = 0.0 

                print("I'm working") 

            print("vel_msg.linear.x: " + str(vel_msg.linear.x)) 
            print("vel_msg.angular.z: " + str(vel_msg.angular.z)) 

            self.cmd_vel_pub.publish(vel_msg) 

            r.sleep()  

    def laser_cb(self, msg):  
        ## This function receives a number   
        #For hls lidar  
        closest_range = min(msg.ranges) 
        idx = msg.ranges.index(closest_range) 
        closest_angle = msg.angle_min + idx * msg.angle_increment 

        self.closest_range = closest_range 
        self.closest_angle = closest_angle 

        #print("closest object distance: " + str(closest_range)) 
        #print("closest object direction: " + str(closest_angle)) 

    def cleanup(self):  
        #This function is called just before finishing the node  
        # You can use it to clean things up before leaving  
        # Example: stop the robot before finishing a node.    
        vel_msg = Twist() 
        self.cmd_vel_pub.publish(vel_msg)  

############################### MAIN PROGRAM ####################################  
if __name__ == "__main__":  
    rospy.init_node("avoid_obstacle", anonymous=True)  
    AvoidObstacleClass() 