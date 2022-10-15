#! /usr/bin/env python

import rospy
import sys

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
from tf.transformations import quaternion_from_euler
from ebot_handler.srv import (arm_go_to_pose, arm_go_to_poseResponse)
from object_msgs.msg import ObjectPose


class Ur5Moveit:

    # Constructor
    def __init__(self):

        #rospy.init_node('node_eg4_set_joint_angles', anonymous=True)

        self._planning_group = "ur5_arm"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        print("Planning Frame:")
        print(self._planning_frame)
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()


    def go_to_pose(self, arg_pose):
        current_robot_state = self._robot.get_current_state()

        self._group.set_start_state(current_robot_state)
        self._group.set_pose_target(arg_pose)
        plan = self._group.plan()
        
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose

        list_joint_values = self._group.get_current_joint_values()

        return flag_plan

    def go_to_predefined_pose(self, arg_pose_name):
        self._group.set_named_target(arg_pose_name)
        plan = self._group.plan()
        # goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        # goal.trajectory = plan
        # self._exectute_trajectory_client.send_goal(goal)
        # self._exectute_trajectory_client.wait_for_result()
        self._group.go(wait=True)


class gripper:

    # Constructor
    def __init__(self):


        self._planning_group = "two_finger_gripper"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

    def go_to_predefined_pose(self, arg_pose_name):
        self._group.set_named_target(arg_pose_name)
        plan = self._group.plan()
        # goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        # goal.trajectory = plan
        # self._exectute_trajectory_client.send_goal(goal)
        # self._exectute_trajectory_client.wait_for_result()
        self._group.go(wait=True)



class WorldObject:
    '''Class to create world object and provide helper functions like pick_object, drop_im_box
        Required parameters:
            name -> type:string -> name of the object
            position -> type:list -> list containing coords of the object
            orientation -> type:list -> list containg orientation of the object in euler'''

    # constructor
    def __init__(self, name, position, orientation, arm, gr):
        self.name = name
        self.position = position
        self.orientation = quaternion_from_euler(orientation[0], orientation[1], orientation[2], axes='sxyz')
        self.ur5 = arm 
        self.gr = gr 

    def pick_object(self):
        '''Function to pick object using Ur5Moveit() and gripper objects'''

        ur5_pose = geometry_msgs.msg.Pose()
        ur5_pose.position.x = self.position[0] - 0.03
        ur5_pose.position.y = self.position[1] - 0.35
        ur5_pose.position.z = self.position[2] + 0.75
        ur5_pose.orientation.x = self.orientation[0]
        ur5_pose.orientation.y = self.orientation[1]
        ur5_pose.orientation.z = self.orientation[2]
        ur5_pose.orientation.w = self.orientation[3]

        # open the gripper before picking the object
        self.gr.go_to_predefined_pose("open")
        self.ur5.go_to_pose(ur5_pose)
        rospy.sleep(1)

        # descend the arm closer to  object position
        ur5_pose.position.z = self.position[2] + 0.65
        ur5_pose.position.y = self.position[1] - 0.20 
        self.ur5.go_to_pose(ur5_pose)

        # close the gripper to hold the object
        self.gr.go_to_predefined_pose("close")

        # ascend the arm to earlier position
        ur5_pose.position.z = self.position[2] + 1.0
        self.ur5.go_to_pose(ur5_pose)


def drop_in_box(ur5, gr):
    ''' function to drop the object in the drop box using predefined drop_pose''' 

    # go to predefined drop_pose
    ur5.go_to_predefined_pose("drop_pose")

    # open the gripper to drop the object in box
    gr.go_to_predefined_pose("open")


def arm_manipulation_cb(cb_data):
    '''callback function for arm manipulation service
        return: success boolean as True when work is done'''

    arm = Ur5Moveit()
    gr = gripper()

    # check the pose name mentioned in the recieved message and move ur5 accordingly
    if cb_data.predefined_pose_name == "pick":
        obj = WorldObject(cb_data.object_name, [cb_data.position.x, cb_data.position.y, cb_data.position.z], [-2.555, 0, 3.1], arm, gr)
        obj.pick_object()
    
    elif cb_data.predefined_pose_name == "drop_pose":
        drop_in_box(arm, gr)

    else:
        arm.go_to_predefined_pose(cb_data.predefined_pose_name)

    return arm_go_to_poseResponse(
        sucess = True
    )
    


def main():
    # initialize a pick_and_place node
    rospy.init_node('pick_and_place')

    # initialize amr_server for arm_manipulation service
    arm_server = rospy.Service('/arm_controller_service', arm_go_to_pose, arm_manipulation_cb)

    rospy.loginfo("Arm Manipulation Service INITIALIZED...")
    rospy.loginfo("Waiting for request from client")

    rospy.spin()


if __name__ == '__main__':
    try:
        main()

    except rospy.ROSInterruptException:
        pass

