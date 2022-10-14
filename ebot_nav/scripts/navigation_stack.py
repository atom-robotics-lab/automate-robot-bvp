#! /usr/bin/env python

import rospy
import time
import numpy as np
import yaml
import io 

# import actionlib & actionlib_msgs to create a SimpleActionClient object 
from actionlib import SimpleActionClient, SimpleGoalState, SimpleActionServer
from actionlib_msgs.msg import GoalStatus

from ebot_handler.msg import NavAction, NavResult, NavFeedback

# import geometry_msgs to create Pose objects with goal coords
from geometry_msgs.msg import Pose, Point, Quaternion

# import nav_msgs for Odometry
from nav_msgs.msg import Odometry

# import quaternion_from_euler function
from tf.transformations import quaternion_from_euler, euler_from_quaternion

# import the move_base messages to send goals 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class WaypointTracker:
    '''A class to store waypoint data and provide the waypoints when needed'''

    def __init__(self, room_name):
        room_waypoints = rospy.get_param('/room-coords/{}'.format(room_name))
        self.waypoints = [eval(pose) for pose in room_waypoints]

        # convert the position and orientation data from param to pose
        self.waypoints = [self.param_to_pose(pose) for pose in self.waypoints]
        self.total_waypoints = len(self.waypoints)
        self.curr = 0

    def param_to_pose(self, point_param):
        point_pose = Pose()
        point_pose.position.x = point_param[0]
        point_pose.position.y = point_param[1]
        point_pose.position.z = 0 
        point_pose.orientation.x = 0
        point_pose.orientation.y = 0
        point_pose.orientation.z = point_param[2] 
        point_pose.orientation.w = point_param[3] 
        return point_pose

    def get_waypoint(self):

        self.curr_waypoint = self.waypoints[self.curr]
        self.curr = self.curr + 1
        return self.curr_waypoint 

    def get_curr_goal(self):
        return self.curr_waypoint

    def get_curr(self):
        return self.curr

    def get_total_waypoints(self):
        return self.total_waypoints

''' MoveBase class responsible for communicating with move_base action server
    and send goals to it'''

class MoveBase:

    def __init__(self, room_name):

        # create NavWaypoints object to supply waypoints
        self.waypoint_supplier = WaypointTracker(room_name)

        # create SimpleActionClient object named 'move_base'
        self.action_client = SimpleActionClient('move_base', MoveBaseAction)

        # wait for action server to start
        self.action_client.wait_for_server()


    ''' move_to():
        method that prepares the goal msg and sends it
        params: 'goal' Pose object with required coords and orientation of goal to be achieved
        return: does not return anything'''

    def move_to(self, goal):

        # initialize a MoveBaseGoal object to create a message
        goal_msg = MoveBaseGoal()

        # set message headers with frame and time
        goal_msg.target_pose.header.frame_id = "map"
        goal_msg.target_pose.header.stamp = rospy.Time.now()

        # set the goal coordinates in message
        goal_msg.target_pose.pose = goal

        # send the message to Action Server in move_base node
        self.action_client.send_goal(goal_msg)
        # print("WAITING FOR GOAL RESUTL")
        # wait = self.action_client.wait_for_result()
        # print("RESULT AA GAYAAA")
        # print(wait)
        # if not wait:
        #     rospy.logerr("Action server not available!")
        #     rospy.signal_shutdown("Action server not available!")
        # else:
        #     print(self.action_client.get_result())
        #     # Result of executing the action
            
        #     return self.action_client.get_result() 

        # if self.action_client.get_state() == SimpleGoalState.DONE:
        #     return True


''' odom_callback():
    callback method to update global pose variable with /odom data''' 

def odom_callback(data):
    global pose
    x  = data.pose.pose.orientation.x;
    y  = data.pose.pose.orientation.y;
    z = data.pose.pose.orientation.z;
    w = data.pose.pose.orientation.w;
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]

            
class NavigationAction:
    '''Class to create and implement SimpleActionServer for navigation'''

    def __init__(self, name):
        self.server_name = name
        self.nav_server = SimpleActionServer(self.server_name, NavAction, self.execute_cb, auto_start=False) 
        self._feedback = NavFeedback()
        self._result = NavResult()
        rospy.loginfo("Starting Navigation Action Server")
        self.nav_server.start()
        rospy.loginfo("Navigation Action Server STARTED")

    def execute_cb(self, goal):
        rospy.loginfo("GOAL recived")
        rate = rospy.Rate(1)

        move_base = MoveBase(goal.room_name)
        #total_waypoints = move_base.waypoint_supplier.get_total_waypoints()
        # get the initial waypoint an call move_to() for it
        initial_waypoint = move_base.waypoint_supplier.get_waypoint()
        move_base.move_to(initial_waypoint)

        dist_precision = 0.2
        theta_precision = 0.15

        try:
            while True:
                global pose
                # get the curr waypoint being approached
                curr_waypoint = move_base.waypoint_supplier.get_curr_goal()
                yaw_final = euler_from_quaternion([curr_waypoint.orientation.x, curr_waypoint.orientation.y, curr_waypoint.orientation.z, curr_waypoint.orientation.w])[2]
                # calculate error in goal and curr position
                position_error = np.sqrt(pow(curr_waypoint.position.y - pose[1], 2) + pow(curr_waypoint.position.x - pose[0], 2))
                theta_error = np.abs(yaw_final) - np.abs(pose[2])
                theta_error = np.abs(theta_error)
                # define and publish the action feedback
                self._feedback.position_error = position_error
                self._feedback.theta_error = theta_error 
                self.nav_server.publish_feedback(self._feedback)
                # check if position_error & theta_error is less than desired precision, then move bot to next goal
                if position_error < dist_precision:
                    if theta_error < theta_precision:
                        self._result.Nav_success = True
                        self._result.room_reached = goal.room_name
                        self.nav_server.set_succeeded(self._result)
                        return
                        # break loop when room reached
                        # if move_base.waypoint_supplier.get_curr() == total_waypoints:
                        #     self._result.Nav_success = True
                        #     self._result.room_reached = goal.room_name
                        #     self.nav_server.set_succeeded(self._result)
                        #     return
                        # print(move_base.waypoint_supplier.get_curr())
                        # move_base.move_to(move_base.waypoint_supplier.get_waypoint())
                rate.sleep()
        except KeyError:
            rospy.logerr("Object was Not Found in the room")
            self._result.Nav_success = False 
            self._result.room_reached = "" 
            self.nav_server.set_succeeded(self._result)



if __name__ == "__main__":
    try:
        time.sleep(1)
        rospy.init_node('ebot_nav')
        rospy.Subscriber('/odom', Odometry, odom_callback)

        # initialize navigation action server
        nav_action_server = NavigationAction('Nav_action_server')

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
