# ROS on Raspberry Pi 4

The goal of this project is to play around with ROS on Raspberry Pi 4 under Ubuntu 20.04.

### Part1: Playing around with rasbag recorded from self-driving car
1. Use ROS to subscribe to the camera topic.
2. Use CvBridge to convert the images from ROS to OpenCV.
3. Use OpenCV to setup a YOLO_v4 pre-trained model to detect objects in the images.
4. Process the output of YOLO_v4 object detector, and then draw bounding boxes on the image.
5. Create a custom message to publish the information of bounding boxes to the ROS topic.
6. Use CvBridge to convert the processed images(with bounding boxes on it) back to the ROS.
7. Publish the processed images and the bounding boxes information.


### Part2: Playing around with Raspberry Pi camera:
1. Publish video frames from Raspberry Pi camera v2 to ROS topic.
2. Subscribe to the Raspberry Pi camera topic.
3. Use Tensorflow Lite to convert the pre-trained YOLOv4-tiny model to `.tflite` file.
4. Use the TFLite model(YOLOv4-tiny, ssd-mobilenet) to do object detection in real time.
5. Display the processed video frames(with bounding boxes on it).


### Environment
- Raspberry Pi 4
- Ubuntu 20.04
- ROS Noetic
- OpenCV 3.4.12
- Python 3.8.5
