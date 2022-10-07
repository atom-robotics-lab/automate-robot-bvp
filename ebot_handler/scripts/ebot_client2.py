#!/usr/bin/env python

import rospy 
import actionlib
import time 
from Utils import TaskStatusCode

from ebot_handler.msg import NavAction, NavGoal, NavFeedback, NavResult, Task, PerceptionAction, PerceptionGoal, PerceptionResult, PerceptionFeedback, TaskStatus
from ebot_handler.srv import arm_go_to_poseRequest, arm_go_to_pose
from Utils import dropbox



class NavActionClient:
    '''A class to create and manage Navigation Action Client'''

    def __init__(self):
        # create navigation action client
        self.action_client = actionlib.SimpleActionClient('Nav_action_server', NavAction)
        rospy.loginfo("ebot_navigation: Waiting for Navigation server ...")

        # wait for navigation action server to come online
        self.action_client.wait_for_server()
        rospy.loginfo("ebot_navigation: Navigation server is online!")

    def go_to(self, room_name):
        '''A function to send room name as goal to navigation action server
            parameter: room_name: String: name of the destination room
            returns: nothing'''

        self.goal = NavGoal()
        self.goal.room_name = room_name 
        self.action_client.send_goal(self.goal, feedback_cb = self.feedback_cb)
        rospy.loginfo("{} (goal) sent for Navigation!".format(room_name))

    def Nav_result(self):
        '''A function to wait for result from navigation action server and return it after receiving
            parameters: no parameter
            returns: Boolean success'''

        rospy.loginfo("Waiting for Success")
        self.action_client.wait_for_result()

        if self.action_client.get_result():
            rospy.loginfo("Result Recieved:")
            self.result = self.action_client.get_result()
            rospy.loginfo(self.result)
            if self.result.Nav_success:
                rospy.loginfo("[RESULT]: {} REACHED".format(self.result.room_reached.upper()))
                return True
            else:
                rospy.logerr("[Result]: error goal abandoned")
                return False
        

    def feedback_cb(self, feedback): 
        rospy.loginfo("Distance from goal : {}".format(feedback.position_error))
        rospy.loginfo("Theta Error : {}".format(feedback.theta_error)) 


class Ur5ManipulationClient:
    '''A class to create and manage Arm Manipulation service client'''

    def __init__(self):
        rospy.loginfo('Waiting for Arm Controller Service...')
        rospy.wait_for_service('arm_controller_service')
        rospy.loginfo('Arm Controller Service Available')

        # initialize arm manipulation service client
        self.service_client = rospy.ServiceProxy('arm_controller_service',arm_go_to_pose)

        # create empty request object
        self.req = arm_go_to_poseRequest()
    
    def go_to_pose(self, pose):
        '''A function to send request to go to a predefined pose to the arm manipulaiton service
            parameter: pose: String: name of the predefined pose
            returns: nothing'''

        self.req.predefined_pose_name = pose
        self.resp = self.service_client(self.req)
        if self.resp == True:
            rospy.loginfo('arm reached {}'.format(pose))

    def pick_object(self, ob_pose):
        '''A function to send pick request to arm manipulation service
            parameters: ob_pose: object data(name and pose) as returned from perception action server: PerceptionResult
            returns: nothing'''

        # set request pose name as pick
        self.req.predefined_pose_name = 'pick'

        # set request object name as name of the object
        self.req.object_name = ob_pose.name

        # define object pose in service request
        self.req.position.x = ob_pose.pose.pose.position.x
        self.req.position.y = ob_pose.pose.pose.position.y
        self.req.position.z = ob_pose.pose.pose.position.z
        self.req.orientation.x = ob_pose.pose.pose.orientation.x
        self.req.orientation.y = ob_pose.pose.pose.orientation.y 
        self.req.orientation.z = ob_pose.pose.pose.orientation.z
        self.req.orientation.w = ob_pose.pose.pose.orientation.w

        self.resp = self.service_client(self.req)
        if self.resp == True:
            rospy.loginfo('{} PICKED'.format(ob_pose.name.upper()))


class TaskManager:
    '''A class that manages executes the task given to ebot by implementing a control loop and 
        communicating with the navigation and perception action server'''

    def __init__(self, nav_client, manipulation_client):

        # initialize subscriber for task_message topic to receive tasks for the ebot
        rospy.Subscriber("/task", Task, self.task_data_cb)

        # initialize publisher to publish the result of the task
        self.task_status_pub = rospy.Publisher('task_status', TaskStatus, queue_size=10)

        self.nav_client = nav_client
        #self.perception_client = perception_client
        self.arm_pose = manipulation_client

        self.ebot_task = []

        rospy.loginfo("TaskManager INITIALIZED")
        rospy.loginfo("ebot_handler: Waiting For TaskPublisher")
        rospy.sleep(1)
        self.publish_status(TaskStatusCode.WAITING_FOR_TASK_PUBLISHER.value)

    def task_data_cb(self, msg):
        rospy.loginfo("New Task RECEIVED")
        rospy.loginfo(msg)
        self.ebot_task = msg 
        self.control_loop()

    def publish_status(self, status):
        status_msg = TaskStatus()
        status_msg.task_status = status
        self.task_status_pub.publish(status)

    def control_loop(self):
        '''A function that implements the main control loop'''

        rospy.loginfo("STARTED RUN")
        rospy.loginfo("Achieving recieved task")

        # publish in progress status on task_status
        self.publish_status(TaskStatusCode.IN_PROGRESS.value)


        # room_try variable to keep track of number of tries bot makes to detect the required object in a room
        # ths allows bot to visit multiple locations in rooms like pantry
        self.room_try = 1

        self.object_status = False
        

        while self.room_try < 3:

            rospy.loginfo("ebot_manipulation: Arm going to TRAVEL POSE")

            # go to travel pose before going to pick room
            self.arm_pose.go_to_pose('travel_pose')

            # go to pick up room and wait for result
            self.nav_client.go_to(self.ebot_task.pick_room)

            # if success is returned by navigation stack, start perception
            if self.nav_client.Nav_result() == True:
                
                if self.ebot_task.pick_room == "Start-Position":
                    rospy.loginfo("MISSION ACCOMPLISHED!")
                    return

                rospy.loginfo("ebot_manipulation: Arm going to DETECT POSE")
                # go to detect pose before starting perception
                self.arm_pose.go_to_pose('detect_pose')

                # send the object to be picked to perception action client and wait for result from perception action server
                #self.perception_client.object_recognition(self.ebot_task.ob_name)
                #self.object_data = self.perception_client.object_result()
                #self.object_status = self.object_data.ob_success

                # if required object is detected, pick it up and break loop
                #if self.object_status:
                    #rospy.loginfo(self.ebot_task.ob_name + " was DETECTED")
                    #rospy.loginfo("Picking Up " + self.ebot_task.ob_name)
                    #self.arm_pose.pick_object(self.object_data.ob_data)
                    #rospy.loginfo("[RESULT]: " + self.ebot_task.ob_name + " OBJECT PICKED")
                    #break
                '''else:
                    self.room_try += 1
                    self.ebot_task.pick_room = self.ebot_task.pick_room[:-1]
                    self.ebot_task.pick_room = self.ebot_task.pick_room + str(self.room_try)
                   ''' 

        #rospy.loginfo("Going to travel pose")
        # go to travel pose before going to the drop room
        #self.arm_pose.go_to_pose('travel_pose')

        # rospy.loginfo("Going to drop room: " + self.ebot_task.drop_room)
        # go to drop room
        # self.nav_client.go_to(self.ebot_task.drop_room)

        # drop the object when bot reaches drop room
        if self.nav_client.Nav_result() == True:
            #self.arm_pose.go_to_pose('drop_pose')
            rospy.loginfo("[RESULT]: {} DROPPED IN {}".format(self.ebot_task.ob_name.upper(), dropbox[self.ebot_task.drop_room]))	
            # publish success task status code
            self.publish_status(TaskStatusCode.SUCCESS.value)
            return


def main():
    # initialize ebot_handler client node
    rospy.init_node('ebot_handler_client')

    # initializing ebot_navigation action client
    ebot_navigation = NavActionClient()

    # send arm to travel pose befor moving
    ebot_manipulation = Ur5ManipulationClient()
    
    ebot_task_manager = TaskManager(ebot_navigation, ebot_manipulation)
    
    rospy.spin()


if __name__ == '__main__':
    main()