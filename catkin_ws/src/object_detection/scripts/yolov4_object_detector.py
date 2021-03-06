#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import time


class ObjectDetector(object):
    def __init__(self):
        self.image = Image()
        self.setup_ros()
        self.setup_yolo()
        self.loop()

    def setup_ros(self):
        """
        Build the node, the subscriber, and the publisher
        """ 

        ### Subscriber ###
        # Create a node named "image_subscriber"
        rospy.init_node("image_subscriber", anonymous=True)
        # Using "rostopic list" to see all the topics
        # Subscribe message to "/image_raw" topic
        rospy.Subscriber("/image_raw", Image, self.image_callback)

        ### Publisher ###
        self.pub = rospy.Publisher('image_processed', Image, queue_size=1)

    def setup_yolo(self):
        # Load names of classes
        self.class_names = []
        with open("/home/think/ros_project/catkin_ws/src/object_detection/weights/classes.txt", "r") as f:
            self.class_names = [cname.strip() for cname in f.readlines()]
        
        # Load a network
        self.net = cv2.dnn.readNet("/home/think/ros_project/catkin_ws/src/object_detection/weights/yolov4-tiny.cfg",
                                   "/home/think/ros_project/catkin_ws/src/object_detection/weights/yolov4-tiny.weights")
        # If OpenCV is compiled with Intel's Inference Engine library, 
        # DNN_BACKEND_DEFAULT means DNN_BACKEND_INFERENCE_ENGINE. 
        # Otherwise it equals to DNN_BACKEND_OPENCV.
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        # Ask network to make computations on specific target device.
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        # Returns names of layers with unconnected outputs.
        self.outNames = self.net.getUnconnectedOutLayersNames()

    def image_callback(self, msg):
        """
        Build the image callback
        """
        self.image = msg

    def action_loop(self):
        """
        Convert between ros and opencv
        """
        try:
            bridge = CvBridge()
            # Convert from ROS to OpenCV
            cv_image = bridge.imgmsg_to_cv2(self.image, desired_encoding='rgb8')

            # Run tiny YOLO
            start = time.time()
            yolo_image = self.get_bounding_boxes(cv_image)
            end = time.time()
            print("FPS:", 1/(end - start))

            # Convert back to ROS and publish
            image_message = bridge.cv2_to_imgmsg(np.asarray(yolo_image), encoding='rgb8')
            return image_message

        except CvBridgeError as e:
            print(e)
            return self.image

    def get_bounding_boxes(self, frame):
        """
        Pass each frame into the yolo network, then draw the bounding boxes
        """
        # Creates 4-dimensional blob from image.
        # (416,416) is the image size we feed into yolo
        blob = cv2.dnn.blobFromImage(frame, scalefactor=1/255, size=(416,416))
        # Sets the new input value for the network.
        self.net.setInput(blob)
        # Runs forward pass to compute output of layer with name outputName.
        outs = self.net.forward(self.outNames)
        
        frameHeight = frame.shape[0]
        frameWidth = frame.shape[1]
        confThreshold = 0.5

        layerNames = self.net.getLayerNames()
        lastLayerId = self.net.getLayerId(layerNames[-1])
        lastLayer = self.net.getLayer(lastLayerId)

        classIds = []
        confidences = []
        boxes = []

        # Using confThershold to filter out not valid detections
        # Then get the class id, confidence, and bounding boxes of the valid detection
        for out in outs:
            for detection in out:
                scores =  detection[5:]
                classId = np.argmax(scores)
                confidence = scores[classId]
                if confidence > confThreshold:
                    center_x = int(detection[0] * frameWidth)
                    center_y = int(detection[1] * frameHeight)
                    width = int(detection[2] * frameWidth)
                    height = int(detection[3] * frameHeight)
                    left = int(center_x - width/2)
                    top = int(center_y - height/2)
                    classIds.append(classId)
                    confidences.append(float(confidence))
                    boxes.append([left, top, width, height])
        
        # Draw the bounding boxes on the frame
        indices = np.arange(0, len(classIds))
        for i in indices:
            box = boxes[i]
            left = box[0]
            top = box[1]
            width = box[2]
            height = box[3]
            self.drawPred(frame, classIds[i], confidences[i], left, top, left+width, top+height)

        return frame

    def drawPred(self, frame, classId, confident, left, top, right, bottom):
        """
        Draw predicted bounding boxes and labels on image
        """
        # Draw a bounding box
        cv2.rectangle(frame, (left, top), (right, bottom), (0,255,0))

        # Set something like "Car: 0.73"
        label = '%.2f' % confident
        if self.class_names:
            assert(classId < len(self.class_names))
            label = '%s: %s' % (self.class_names[classId], label)
        
        # Calculates the width and height of a text string.
        # cv2.getTextSize(text, fontFace, fontScale, thickness)
        labelSize, baseLine = cv2.getTextSize(label,cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        top = max(top, labelSize[1])
        cv2.rectangle(frame, (left, top - labelSize[1]), (left + labelSize[0], top + baseLine), (255, 255, 255))
        cv2.putText(frame, label, (left, top), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0))


    def loop(self):
        """
        Public messages
        """
        # go through the loop 15 times per second
        rate = rospy.Rate(15) # 15hz

        while not rospy.is_shutdown():
            processed_image = self.action_loop()
            self.pub.publish(processed_image)
            # the loop calls rate.sleep(), which sleeps just long enough to 
            # maintain the desired rate through the loop.
            rate.sleep()

if __name__ == '__main__':
    try:
        ObjectDetector()
    except rospy.ROSInterruptException:
        pass