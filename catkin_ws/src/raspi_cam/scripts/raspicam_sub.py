#!/usr/bin/env python3
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge, CvBridgeError # Package to convert between ROS and OpenCV Images
import cv2
import numpy as np
import  tensorflow as tf
import time

class CameraSubscriber(object):
    def __init__(self):
        self.receive_message()

    def receive_message(self):
        # Tells rospy the name of the node.
        rospy.init_node('camera_subscriber', anonymous=True)

        # Subscribing to the video_frames topic
        rospy.Subscriber('/video_frames', Image, self.callback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

        # Close down the video stream when done
        cv2.destroyAllWindows()

    def callback(self, msg):
        try:
            br = CvBridge()
            
            # Output debugging information to the terminal
            rospy.loginfo("receiving video frame")

            # Convert ROS Image message to OpenCV image
            current_frame = br.imgmsg_to_cv2(msg, desired_encoding='rgb8')

            # Start timer (for calculating frame rate)
            t1 = cv2.getTickCount()

            # Do object detection
            processed_frame = self.object_detection(current_frame)

            # Calculate framerate
            t2 = cv2.getTickCount()
            time1 = (t2-t1) / cv2.getTickFrequency()
            frame_rate= 1 / time1

            # Draw framerate in corner of frame
            cv2.putText(processed_frame, 'FPS: {0:.2f}'.format(frame_rate), (30,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,0), 2, cv2.LINE_AA)

            # Display image
            cv2.imshow("raspi_cam", current_frame)
            cv2.waitKey(1)

        except CvBridgeError as e:
            print(e)

    def object_detection(self, frame):
        # Load TensorFlow Lite Models and allocate tensors.
        interpreter = tf.lite.Interpreter(model_path="/home/think/ros_project/catkin_ws/src/raspi_cam/Sample_TFLite_model/detect.tflite")
        interpreter.allocate_tensors()

        # Load the label map
        with open("/home/think/ros_project/catkin_ws/src/raspi_cam/Sample_TFLite_model/labelmap.txt", 'r') as f:
            labels = [line.strip() for line in f.readlines()]

        # Have to do a weird fix for label map if using the COCO "starter model" from
        # https://www.tensorflow.org/lite/models/object_detection/overview
        # First label is '???', which has to be removed.
        if labels[0] == '???':
            del(labels[0])

        # Get input and output tensors.
        # [{'name': 'conv2d_input', 'index': 8, 'shape': array([ 1, 28, 28,  1]), 
        #   'dtype': <class 'numpy.float32'>, 'quantization': (0.0, 0)}]
        input_details = interpreter.get_input_details()
        output_details = interpreter.get_output_details()

        # Get the width and height of the input image of the model
        _, height, width, _ = input_details[0]['shape']

        # Acquire image and resize to expected shape [1xHxWx3]
        img = frame.copy()
        imH, imW, _ = img.shape
        image_resized = cv2.resize(img, (width, height))
        input_data = np.expand_dims(image_resized, axis=0)

        # Check if the model is quantized or not
        floating_model = input_details[0]['dtype'] == np.float32

        # Normalize pixel values if using a floating model (i.e. if model is non-quantized)
        if floating_model:
            # input_data = np.float32(input_data) / 255.
            input_data = (np.float32(input_data) - 127.5) / 127.5

        # Perform the actual detection by running the model with the image as input
        # Sets the value of the input tensor.
        interpreter.set_tensor(input_details[0]['index'], input_data)

        # Invoke the interpreter.
        # Be sure to set the input sizes, allocate tensors and fill values before calling this.
        interpreter.invoke()

        # Retrieve detection results
        # Bounding box coordinates of detected objects
        boxes = interpreter.get_tensor(output_details[0]['index'])[0] 
        # Class index of detected objects
        classes = interpreter.get_tensor(output_details[1]['index'])[0] 
        # Confidence of detected objects
        scores = interpreter.get_tensor(output_details[2]['index'])[0]
        # Total number of detected objects (inaccurate and not needed)
        # num = interpreter.get_tensor(output_details[3]['index'])[0]

        min_conf_threshold = 0.5
        # Loop over all detections and draw detection box if confidence is above minimum threshold
        for i in range(len(scores)):
            if ((scores[i] > min_conf_threshold) and (scores[i] <= 1.0)):

                # Get bounding box coordinates and draw box
                # Interpreter can return coordinates that are outside of image dimensions, 
                # need to force them to be within image using max() and min()
                ymin = int(max(1,(boxes[i][0] * imH)))
                xmin = int(max(1,(boxes[i][1] * imW)))
                ymax = int(min(imH,(boxes[i][2] * imH)))
                xmax = int(min(imW,(boxes[i][3] * imW)))
                
                cv2.rectangle(frame, (xmin,ymin), (xmax,ymax), (10, 255, 0), 2)

                # Draw label
                # Look up object name from "labels" array using class index
                object_name = labels[int(classes[i])] 
                # Example: 'person: 72%'
                label = '%s: %d%%' % (object_name, int(scores[i]*100))
                # Get font size
                labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
                # Make sure not to draw label too close to top of window
                label_ymin = max(ymin, labelSize[1] + 10)
                # Draw white box to put label text in
                cv2.rectangle(frame, (xmin, label_ymin-labelSize[1]-10), (xmin+labelSize[0], label_ymin+baseLine-10), (255, 255, 255), cv2.FILLED)
                # Draw label text
                cv2.putText(frame, label, (xmin, label_ymin-7), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)

        return frame

if __name__ == '__main__':
    try:
        CameraSubscriber()
    except rospy.ROSInterruptException:
        pass