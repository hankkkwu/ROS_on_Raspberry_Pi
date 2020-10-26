# ROS on Raspberry Pi 4

The goal of this project is to play around with ROS on Raspberry Pi 4 under Ubuntu 20.04.

What I did in this project is:
1. Use ROS to subscribe to the camera topic.
2. Use CvBridge to convert the images from ROS to OpenCV.
3. Use OpenCV to setup a YOLO_v4 object detector to detect objects in the images.
4. Process the output of YOLO_v4 object detector, and then draw bounding boxes on the image.
5. Create a custom message to publish the information of bounding boxes to the ROS topic.
6. Use CvBridge to convert the processed images(with bounding boxes on it) back to the ROS.
7. Publish the processed images and the bounding boxes information.

# Environment
Raspberry Pi 4
Ubuntu 20.04
ROS Noetic
OpenCV 3.4.12
Python 3.8.5
