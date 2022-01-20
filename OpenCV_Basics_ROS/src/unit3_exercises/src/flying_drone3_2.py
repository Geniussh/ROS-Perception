#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

class Flying():

    def __init__(self):
        self.hector_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.hector_sub = rospy.Subscriber("/sonar_height", Range, self.scan_callback)
        
        self.cmd = Twist()
        self.ctrl_c = False
        self.rate = rospy.Rate(10) # 10hz
        self.a = 0.0
        rospy.on_shutdown(self.shutdownhook)

    def scan_callback(self, msg):
        self.a = msg.range 
         
    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.ctrl_c = True

    def move_drone(self):
        while not self.ctrl_c:

            if self.a < 1.8:
                self.cmd.linear.z = 0.12
                
            if self.a > 1.93:
                self.cmd.linear.z = 0.0

            if self.a < 0.5:
                self.cmd.linear.x = -1.0
            if self.a > 0.51: 
                self.cmd.linear.x = 0.0
            
            self.hector_vel_publisher.publish(self.cmd)   

if __name__ == '__main__':
    rospy.init_node('hector_test', anonymous=True)
    hector_object = Flying()
    try:
        hector_object.move_drone()
    except rospy.ROSInterruptException:
        pass