#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion
import numpy as np
from master_controller.msg import obj_confirmation

# create a global state variable to store the current state of the goto state machine
state = 1

# create velocity_msg Twist object to publish messages to /cmd_vel topic
velocity_msg = Twist()

# declare a global publisher variable
pub = None

d_state = 0

distance = 0

w_state = 0

def change_state(m_state):
    global state
    state = m_state


def move(linear,angular):
	global pub,velocity_msg
	velocity_msg.linear.x = linear
	velocity_msg.angular.z = angular
	pub.publish(velocity_msg)

def change_state(m_state):
    global state
    state = m_state
    move(0,0)

def check_wall():
    global distance, w_state
    dmin = 0.9
    if distance < dmin:
	    w_state = 1
		

def laser_callback(msg):
    global distance
    distance = min(min(msg.ranges[320:400]), 10)
    check_wall()


def rotate():
    global distance, w_state
    dmax = 1.3
    dmid = 1.0
    rate = rospy.Rate(10)
    while distance < dmid:
        move(-0.25, 0)

    while distance < dmax:
        move(0, 1)
        
    
    w_state = 0



def control_loop():
    global pub, velocity_msg, state, w_state

    rospy.init_node('ebot_controller')

    # initialize publisher to publish to /cmd_vel topic
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # subscribe to /odom and /ebot/laser/scan
    rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)

    rospy.Subscriber('finding_object', obj_confirmation, change_state)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if state == 1:
            if w_state == 0:
                move(0.5,0)
            elif w_state == 1:
                rotate()
    
        rate.sleep()
    rospy.spin()
        


if __name__ == "__main__":
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass

