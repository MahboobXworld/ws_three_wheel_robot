#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

rospy.init_node('my_keyboard_controller')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

rate = rospy.Rate(10)
twist = Twist()

# Example: move forward
twist.linear.x = 0.5
twist.angular.z = 0.0

while not rospy.is_shutdown():
    pub.publish(twist)
    rate.sleep()
