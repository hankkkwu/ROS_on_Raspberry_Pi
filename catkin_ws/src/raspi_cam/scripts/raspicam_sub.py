#!/usr/bin/env python3
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge, CvBridgeError # Package to convert between ROS and OpenCV Images
import cv2

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
            current_frame = br.imgmsg_to_cv2(msg)

            # Display image
            cv2.imshow("raspi_cam", current_frame)
            cv2.waitKey(1)

        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    try:
        CameraSubscriber()
    except rospy.ROSInterruptException:
        pass