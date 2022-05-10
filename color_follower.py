#!/usr/bin/env python3
import rospy 
from std_msgs.msg import Int32  
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist

# This class will receive the radius and center of the detected object
# and make the robot move towards it.
class ColorFollowerClass(): 
    ############ CONSTANTS ################
    # Screen width = 600, xm = 300
    xm = 300
    kv = 7.5
    kw = 0.005
    def __init__(self): 
        rospy.on_shutdown(self.cleanup) 

        ###******* INIT PUBLISHERS *******### 
        ##  pub = rospy.Publisher('setPoint', UInt16MultiArray, queue_size=1) 
        self.cmd_vel_pub= rospy.Publisher('cmd_vel', Twist, queue_size=1) 

        ###############ColorFollowerClass################ SUBSCRIBERS ##################################### 
        rospy.Subscriber("radius", Int32, self.radius_cb) 
        rospy.Subscriber("center", Point, self.center_cb) 

        
        

        ############ VARIABLES ################ 

        self.vel = Twist()
        self.radius = 0
        self.center = Point()
        thres_x = 5.5
        thres_w = 5.5

        #********** INIT NODE **********### 
        Hz = 10
        r = rospy.Rate(Hz) #10Hz 
        print("Node initialized (" + str(Hz) + "Hz)")

        self.vel = Twist()

        while not rospy.is_shutdown():
            ex = self.xm - self.center.x
            ew = self.xm - self.radius
            
            if (abs(ex) >= thres_x):
                self.vel.angular.z = self.kw * ex
            elif (abs(ew) >= thres_w and self.radius > 0):
                if (0 < abs(ew) <=0.5):
                    self.vel.linear.x = 0.0 
                    self.vel.angular.z = 0.0
                else:
                    self.vel.angular.z = 0.0
                    self.vel.linear.x = self.kv * 1 / self.radius

            self.cmd_vel_pub.publish(self.vel) #publish the speed 
            print(self.vel)
            r.sleep() 

    def radius_cb(self, radius): 
        ## This function receives a number  
        self.radius = radius.data
        pass 
        
    def center_cb(self, center): 
        ## This function receives a number.
        self.center = center
        pass 
        
    def cleanup(self): 
        #This function is called just before finishing the node 
        # You can use it to clean things up before leaving 
        # Example: stop the robot before finishing a node. 
        print('\nProcess Terminated')
        pass 
############################### MAIN PROGRAM #################################### 
if __name__ == "__main__": 
    rospy.init_node("adder_node", anonymous=True) 
    ColorFollowerClass() 
