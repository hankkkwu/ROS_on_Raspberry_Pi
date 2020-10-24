#!/usr/bin/env python3
import rospy
# Run "rosmsg list" to pick the messages we need
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError
import cv2

class ObjectDetector(object):
    def __init__(self):
        self.image = Image()
        self.setup_ros()
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

            # Do something using OpenCV
            grayscale_img = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)

            # Convert back to ROS and publish
            image_message = bridge.cv2_to_imgmsg(grayscale_img)
            return image_message

        except CvBridgeError as e:
            print(e)
            return self.image

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