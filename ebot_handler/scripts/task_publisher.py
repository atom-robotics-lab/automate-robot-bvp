#! /usr/bin/env python

import rospy
from ebot_handler.msg import Task, TaskStatus
from Utils import TaskStatusCode

class TaskPublisher:
    '''Class to get task details and publish it to ebot_handler client as per the task status code received'''
    
    def __init__(self):
        # initialize publisher to publish task message
        self.task_pub = rospy.Publisher('/task', Task)
        # initialize subscriber for task status
        self.task_result_sub = rospy.Subscriber('/task_status', TaskStatus, self.task_status_cb)

        rospy.loginfo('Getting Task Checkpoints...')
        # get task checkpoints from params
        self.task_checkpoints = rospy.get_param('task-checkpoints')
        self.curr_checkpoint = 1

    def task_status_cb(self, msg):
        # if task status received is success, fail or waiting the publish next task
        if TaskStatusCode(msg.task_status).name in ['SUCCESS', 'FAIL', 'WAITING_FOR_TASK_PUBLISHER']:
            if self.curr_checkpoint > 4:
                rospy.logwarn('No More Tasks Left!')
                return

            # get next checkpoint from task params when ebot_handler status is success, fail, or waiting
            rospy.loginfo("Received Task Status: " + TaskStatusCode(msg.task_status).name) 
            curr_checkpoint = 'checkpoint-' + str(self.curr_checkpoint)
            pick_room = self.task_checkpoints[curr_checkpoint]['pick_room']
            pick_object = self.task_checkpoints[curr_checkpoint]['ob_name']
            drop_room = self.task_checkpoints[curr_checkpoint]['drop_room']

            # publish the required task
            self.publish_task(pick_room, pick_object, drop_room)

            self.curr_checkpoint += 1

        else:
            rospy.loginfo("Received Task Status: " + TaskStatusCode(msg.task_status).name) 

    def publish_task(self, pick_room, pick_ob, drop_room):
        '''A function to create task message and publish it on task topic
            parameters: pick_room: String: room name
                        pick_ob: String: object name
                        drop_room: String: room name'''

        task_msg = Task()

        task_msg.pick_room = pick_room
        task_msg.ob_name = pick_ob
        task_msg.drop_room = drop_room 

        self.task_pub.publish(task_msg)

def main():
    rospy.init_node('task_publisher')

    task_publisher = TaskPublisher()

    rospy.spin()


if __name__ == "__main__":
    main()
