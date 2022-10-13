#!/usr/bin/env python

import cv2
import time
import sys
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image, CompressedImage
import os
import rospy
<<<<<<< HEAD

from geometry_msgs.msg import PoseStamped
from object_msgs.msg import ObjectPose
from sensor_msgs.msg import Image
from find_object_2d.msg import ObjectsStamped, DetectionInfo

from actionlib import SimpleActionServer
from ebot_handler.msg import PerceptionAction, PerceptionResult, PerceptionFeedbacks
from rospy.exceptions import ROSException

import roslaunch

import message_filters

=======
>>>>>>> yolo working on robotic arm's camera

INPUT_WIDTH = 640
INPUT_HEIGHT = 640
SCORE_THRESHOLD = 0.2
NMS_THRESHOLD = 0.4
CONFIDENCE_THRESHOLD = 0.8

ROOT_DIR = os.getcwd()


colors = [(255, 255, 0), (0, 255, 0), (0, 255, 255), (255, 0, 0)]

is_cuda = len(sys.argv) > 1 and sys.argv[1] == "cuda"

#net = build_model(is_cuda)
#capture = load_capture()

start = time.time_ns()
frame_count = 0
total_frames = 0
fps = -1


class WorkpieceDetector :
    def __init__(self, ob_name):
        self.frame_count = 0
        self.total_frames = 0
        self.fps = -1
        self.start = time.time_ns()
        #self.frame = frame
        self.bridge = CvBridge()
<<<<<<< HEAD
        self.image_sub = rospy.Subscriber("/camera/color/image_raw2/compressed", CompressedImage, self.load_capture)
        self.object = ob_name
        self.bb_frame = None
=======
        self.image_sub = rospy.Subscriber("/camera/color/image_raw2/compressed",CompressedImage,self.load_capture)
>>>>>>> yolo working on robotic arm's camera
        
        

    def build_model(self , is_cuda):
        self.net = cv2.dnn.readNet("src/automate-robot-bvp/ebot_perception/scripts/utils/automate.onnx")
        if is_cuda:
            print("Attempty to use CUDA")
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)
        else:
            print("Running on CPU")
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        return self.net



    def detect(self,image,net ):
        self.net = net
        blob = cv2.dnn.blobFromImage(image, 1/255.0, (INPUT_WIDTH, INPUT_HEIGHT), swapRB=True, crop=False)
        self.net.setInput(blob)
        preds = self.net.forward()
        return preds

    def load_capture(self, data):
        
        #self.capture = cv2.VideoCapture("sample.mp4")

        np_arr = np.fromstring(data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.frame = image_np
        #self.frame = self.bridge.imgmsg_to_cv2(data)
        self.capture = self.frame
        #return self.capture
        self.control_loop()

    def load_classes(self):
        self.class_list = []
        with open("src/automate-robot-bvp/ebot_perception/scripts/utils/classes.txt", "r") as f:
            self.class_list = [cname.strip() for cname in f.readlines()]
        return self.class_list



    def wrap_detection(self,input_image, output_data):
        class_ids = []
        confidences = []
        boxes = []

        rows = output_data.shape[0]

        image_width, image_height, _ = input_image.shape

        x_factor = image_width / INPUT_WIDTH
        y_factor =  image_height / INPUT_HEIGHT

        for r in range(rows):
            row = output_data[r]
            confidence = row[4]
            if confidence >= CONFIDENCE_THRESHOLD:

                classes_scores = row[5:]
                _, _, _, max_indx = cv2.minMaxLoc(classes_scores)
                class_id = max_indx[1]
                if (classes_scores[class_id] > .25):

                    confidences.append(confidence)

                    class_ids.append(class_id)

                    x, y, w, h = row[0].item(), row[1].item(), row[2].item(), row[3].item() 
                    left = int((x - 0.5 * w) * x_factor)
                    top = int((y - 0.5 * h) * y_factor)
                    width = int(w * x_factor)
                    height = int(h * y_factor)
                    box = np.array([left, top, width, height])
                    boxes.append(box)

        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.25, 0.45) 

        result_class_ids = []
        result_confidences = []
        result_boxes = []

        for i in indexes:
            result_confidences.append(confidences[i])
            result_class_ids.append(class_ids[i])
            result_boxes.append(boxes[i])

        return result_class_ids, result_confidences, result_boxes

    def format_yolov5(self,frame):

        row, col, _ = frame.shape
        _max = max(col, row)
        result = np.zeros((_max, _max, 3), np.uint8)
        result[0:row, 0:col] = frame
        return result

    def display_objects() :
        
        if self.bb_frame is not None :
            cv2.imshow("Object Detection", self.bb_frame)
            if cv2.waitKey(100) & 0xFF == ord('q'):
                return

        else:
            rospy.loginfo("Bounding box frame is None")




    def control_loop(self) :
        
        self.net = self.build_model(is_cuda)
        self.load_classes()
<<<<<<< HEAD
        self.objectid = self.class_list.index(self.object)
=======
>>>>>>> yolo working on robotic arm's camera
        #self.capture = self.load_capture()

        #while self.capture is not None:
        frame = self.capture
        print("control_loop")
        
        if frame is None:
            print("End of stream")
            exit()

        try : 
        
            inputImage = self.format_yolov5(frame)
            #resized = cv2.resize(inputImage , (640,640))
            #blurred = cv2.blur(resized ,(10,10))
            outs = self.detect(inputImage, self.net)
            class_ids, confidences, boxes = self.wrap_detection(inputImage, outs[0])
            #print("ID : " , class_ids)
            #print("Boxes : ",boxes)
            self.frame_count += 1
            self.total_frames += 1

            index = None
            return_val = None
            self.bb_frame = frame.copy()

            if self.objectid in class_ids :
                index = class_ids.index(self.objectid)
                classid, confidence, box = class_ids[index], confidences[index], boxes[index]
                color = colors[int(classid) % len(colors)]
                cv2.rectangle(self.bb_frame , box, color, 2)
                cv2.rectangle(self.bb_frame , (box[0], box[1] - 20), (box[0] + box[2], box[1]), color, -1)
                try :
                    cv2.putText(self.bb_frame , self.class_list[classid], (box[0], box[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,0,0))
                except :
                    pass
                return_val = box


            if self.frame_count >= 30:
                self.end = time.time_ns()
                self.fps = 1000000000 * frame_count / (self.end - self.start)
                self.frame_count = 0
                self.start = time.time_ns()
            if self.fps > 0:
                self.fps_label = "FPS: %.2f" % self.fps
                cv2.putText(frame, fps_label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            #cv2.imshow("output", frame)

            #if cv2.waitKey(100) & 0xFF == ord('q'):
                #return

            #print(return_val)
            return return_val

        except :
            return None


class PerceptionActionServer: 
    '''class to generate ObjectPose message and publish it on detection_info topic'''

    def __init__(self, name): 
        self.server_name = name
        self.perception_server = SimpleActionServer(self.server_name, PerceptionAction, self.execute_cb, auto_start=False)
        self._feedback = PerceptionFeedback()
        self._result = PerceptionResult()

        rospy.loginfo("Starting Perception Action Server")
        self.perception_server.start()
        rospy.loginfo("Perception Action Server STARTED")
        
    def execute_cb(self, goal): 
        '''Execute callback function called when goal received by Perception Action Server
            parameters: goal->PerceptionAction->goal sent by Perception Action Client
            returns: nothing'''

        rospy.loginfo("GOAL recieved")

        # wait for find_object_2d to start publishing on /info topic
        # Send false goal if timeout is reached
        try:
            rospy.wait_for_message('/info', DetectionInfo, rospy.Duration(3))
        except ROSException:
            rospy.logerr("WAITING FOR /info MESSAGE TIMEOUT!")
            rospy.logwarn("Maybe find_object_2d has not started fully")
            rospy.logwarn("SENDING FALSE SUCCESS TO CLIENT")
            self._result.ob_success = False
            #self._result.ob_data = ObjectPose()
            self.perception_server.set_succeeded(self._result)
            return

        # initialize ObjectPerception object if message is received on /info to initialize image_sub and ob_sub
        self.ob_perception = WorkpieceDetector(goal.ob_name)

        # wait 3 seconds to allow subscriber to make connections
        rospy.sleep(3)

        # get object data published from find_object_2d 
        ob_data = self.ob_perception.control_loop()

        # if no object is detected return False success to client
        if ob_data is None:
            rospy.logerr("NO OBJECT DETECTED BY FIND_OBJECT_2D")
            self._result.ob_success = False
            self._result.ob_data = ObjectPose()
            self.perception_server.set_succeeded(self._result)
            return

        # display image with bounding box on objects
        self.ob_perception.display_objects()

        ob_detected = 0 

        # get the pose of the object to be picked

        
        
        rospy.loginfo("[RESULT]: {} IDENTIFIED".format(goal.ob_name.upper()))
        #if ob['name'] == goal.ob_name:
        self.msg = ObjectPose()
        self.msg.name = goal.ob_name 
        self.msg.pose = PoseStamped()
        self.msg.pose.pose.position.x = 0 #ob['trans'][0] 
        self.msg.pose.pose.position.y = 0 #ob['trans'][1]
        self.msg.pose.pose.position.z = 0 #ob['trans'][2]
        self.msg.pose.pose.orientation.x = 0 #ob['rot'][0] 
        self.msg.pose.pose.orientation.y = 0 #ob['rot'][1]
        self.msg.pose.pose.orientation.z = 0 #ob['rot'][2]
        self.msg.pose.pose.orientation.w = 0 #ob['rot'][3]
        ob_detected = 1 

            #else:
                #self._feedback.ob_detected = ob['name']
                #self.perception_server.publish_feedback(self._feedback)

        # return True success and ObjectPose message of the object to be picked if it was detected
        # else return false success and empty pose
        if ob_detected:
            self._result.ob_success = True
            self._result.ob_data = self.msg
            self.perception_server.set_succeeded(self._result)
            return

        else:
            self._result.ob_success = False
            #self._result.ob_data = ObjectPose()
            self.perception_server.set_succeeded(self._result)
            return
        

#print("Total frames: " + str(total_frames))

if __name__ == "__main__" :

<<<<<<< HEAD
    try : 
        rospy.init_node("perception_yolo")
        perception_action_server = PerceptionActionServer('Perception_action_server')
        rospy.spin()    
    
    except rospy.ROSInterruptException:
        pass
    
=======
    wd = WorkpieceDetector()
    rospy.spin()
    wd.control_loop()

>>>>>>> yolo working on robotic arm's camera

