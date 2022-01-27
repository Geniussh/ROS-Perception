#! /usr/bin/env python
import rospy
from std_msgs.msg import String

class ObjectivePub():
    def __init__(self):
        self.pub = rospy.Publisher('/objective', String, queue_size=1)
        while self.pub.get_num_connections() < 1:  # wait for publisher connection
            continue

if __name__=='__main__':
    rospy.init_node('objective_publisher')
    pub = ObjectivePub()