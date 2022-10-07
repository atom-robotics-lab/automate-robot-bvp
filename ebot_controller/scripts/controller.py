#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion
import numpy as np

# create a global state variable to store the current state of the goto state machine
state = 0

# global pose variable to store bot pose data
pose = []

# create velocity_msg Twist object to publish messages to /cmd_vel topic
velocity_msg = Twist()

# declare a global publisher variable
pub = None

d = 1.0

dmax = 2.5

d_state = 0

regions = {
        'right': 0,
        'fright': 0,
        'front': 0,
        'fleft': 0,
        'left': 0,
}

w_state = 0

def move(linear,angular):
	global pub,velocity_msg
	velocity_msg.linear.x = linear
	velocity_msg.angular.z = angular
	pub.publish(velocity_msg)

def waypoints(res):

    # create a numpy array of 'res' points in range 0 to 2*pi
    x = np.linspace(0, 2*np.pi, num = res, endpoint=True)[1:]

    # create a numpy array of given path function 
    y = 2 * np.sin(x) * np.sin(x/2)

    return [x, y]


def check_wall():
	global regions, w_state
	if regions['front'] < dmax or regions['fleft'] < dmax or regions['fright'] < dmax:
		w_state = 1
		#rospy.loginfo('wall detected')


def avoid_wall():
	wall_state = ''

	if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
		wall_state = ' nothing'
		move(0.3,-0.7)
        #goto(12,12)
	elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d:
		wall_state = 'front'
		move(0,2.0)
	elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
		wall_state = 'fright'
		move(0,1.5)
	elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d:
		wall_state = 'fleft'
		move(0.3,-0.9)
	elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
		wall_state = 'front and fright'
		move(0,2.0)
	elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:
		wall_state = 'front and fleft'
		move(0,2.0)
	elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:
		wall_state = 'ront and fleft and fright'
		move(0,2.0)
	elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d:
		wall_state = 'fleft and fright'
		move(0.3,-0.9)
	print '%s' % (wall_state)



def laser_callback(msg):
    global regions
    regions = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:713]), 10),
    }
    check_wall()


def odom_callback(data):
    global pose
    x  = data.pose.pose.orientation.x;
    y  = data.pose.pose.orientation.y;
    z = data.pose.pose.orientation.z;
    w = data.pose.pose.orientation.w;
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]


# function to orient the bot towards the destination using Proportional controller
def fix_yaw(error_a, P):

    move(0.1 * np.abs(error_a), P * -error_a)
    #move(0, P * -error_a)

# function to move on a strainght line towards the goal using Proportional controller
def move_straight(error, P):
    
    move(P * error, 0)


'''
    This function moves the bot towards the goal coordinates.
    The function uses a state machine with three states: 
    1) state = 0; fixing yaw 2) state = 1; moving straight 3) state = 2; goal reached 
'''
def goto(dest_x, dest_y):
    print('goto ke andar')
    global state, pose

    # the required precision va;ues for required theta and distance from goal
    theta_precision = 0.16  
    dist_precision = 0.35


    # while current state is not 2 (goal is not reached)
    while state != 2:

        theta_goal = np.arctan((dest_y - pose[1])/(dest_x - pose[0]))
        if theta_goal>0:
        	theta_goal+=0.04
        elif theta_goal<0:
        	theta_goal-=0.04
        bot_theta = pose[2]

        theta_error = round(bot_theta - theta_goal, 2)
        rospy.loginfo("STATE: " + str(state))
        rospy.loginfo("THETA ERROR:" + str(theta_error))

        # if current state is 0 means bot is not correctly oriented
        if state == 0:

            # if theta_error is greated than the required precision then fix the yaw by rotating the bot
            # if required precision is reached then change current state to 1

            if np.abs(theta_error) > theta_precision:   
                rospy.loginfo("Fixing Yaw")
                fix_yaw(theta_error, 1.7)
            else:
                rospy.loginfo("Yaw Fixed! Moving Towards Goal Now")
                state = 1

        # if current state is 1 means the bot is correctly oriented so it move towards goal
        elif state == 1:

            # calculate error w.r.t to destination
            position_error = np.sqrt(pow(dest_y - pose[1], 2) + pow(dest_x - pose[0], 2))
            rospy.loginfo("POSITION ERROR: " + str(position_error))

            # if position error is less than required precision & bot is facing the goal, move towards goal in straight line
            # else if it is not correctly oriented change state to 1
            # if theta_precision and dist_precision are reached change state to 2 (goal reached)
            if position_error > dist_precision and np.abs(theta_error) < theta_precision:
                rospy.loginfo("Moving Straight")
                move_straight(position_error, 0.8)
            elif np.abs(theta_error) > theta_precision: 
                rospy.loginfo("Going out of line!")
                state = 0
            elif position_error < dist_precision:
                rospy.loginfo("GOAL REACHED")
                state = 2
    
def chk_dist():
    global pose,d_state
    distf = 2.5
    if distf > np.sqrt(pow(0 - pose[1], 2) + pow(12.5 - pose[0], 2)):
        d_state = 1
        rospy.loginfo('This is 1')


def control_loop():
    global pub, velocity_msg, state, w_state, d_state, pose

    rospy.init_node('ebot_controller')

    # initialize publisher to publish to /cmd_vel topic
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # subscribe to /odom and /ebot/laser/scan
    rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)

    rate = rospy.Rate(10)

    # publish 0 linear.x and 0 angular.z velocity to stop the bot initially
    move(0,0)

    # get the list of 10 x and y coords for the waypoints
    path_x, path_y = waypoints(1000)

    path_waypoints = zip(path_x, path_y)
    rospy.loginfo(path_waypoints)
    rospy.loginfo(len(path_waypoints))
    
    while not rospy.is_shutdown():

        
    # is global pose list is not empty
     if pose:   
        if d_state == 0:
            if w_state == 0:
            # loop to move the bot to every point in the path_waypoints list
                for (x, y) in path_waypoints:
                    state = 0
                    rospy.loginfo("Moving to point: " + str(x) + "," + str(y))
                    goto(round(x, 2), round(y, 2))
                    if w_state != 0:
                        break
            if w_state == 1:
                avoid_wall()
                chk_dist()
        if d_state == 1:
            state=0
            goto(12.5,0.17)
            #move(0,0)
            break
        rate.sleep()
    move(0,0) 


if __name__ == "__main__":
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass

