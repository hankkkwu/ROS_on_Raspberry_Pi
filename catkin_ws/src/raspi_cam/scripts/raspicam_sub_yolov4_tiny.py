#!/usr/bin/env python3
import rospy # Python library for ROS
from sensor_msgs.msg import Image as ImageMsg # Using ImageMsg to avoid name conflict with PIL's Image
from cv_bridge import CvBridge, CvBridgeError # Package to convert between ROS and OpenCV Images
import cv2
import numpy as np
import tensorflow as tf
import random # for draw_bbox function
import colorsys # for draw_bbox function
from PIL import Image

class CameraSubscriber(object):
    def __init__(self):
        self.receive_message()
        
    def receive_message(self):
        # Tells rospy the name of the node.
        rospy.init_node('camera_subscriber', anonymous=True)
        
        # Subscribing to the video_frames topic
        rospy.Subscriber('/video_frames', ImageMsg, self.callback)
        
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

        # Close down the video stream when done
        cv2.destroyAllWindows()

    def callback(self, msg):
        try:
            br = CvBridge()

            # Output debugging information to the terminal
            rospy.loginfo("receiving video frame..............")

            # Convert ROS Image message to OpenCV image
            current_frame = br.imgmsg_to_cv2(msg, desired_encoding='rgb8')

            # Start timer (for calculating frame rate)
            t1 = cv2.getTickCount()

            # Do object detection
            processed_frame = object_detection(current_frame)

            # Calculate framerate
            t2 = cv2.getTickCount()
            time1 = (t2-t1) / cv2.getTickFrequency()
            frame_rate= 1 / time1
            rospy.loginfo("Finished processed..............")
            print("FPS: ", frame_rate)

            # Draw framerate in corner of frame
            cv2.putText(processed_frame, 'FPS: {0:.2f}'.format(frame_rate), (30,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,0), 2, cv2.LINE_AA)

            # Display image
            cv2.imshow("raspi_cam", processed_frame)
            cv2.waitKey(1)

        except CvBridgeError as e:
            print(e)

def object_detection(frame):
    iou_threshold = 0.45
    score_threshold = 0.25
    input_size = 416

    # Acquire image and resize to expected shape [1xHxWx3]
    image_data = cv2.resize(frame, (input_size, input_size))
    image_data = image_data / 255.
    images_data = []
    for i in range(1):
        images_data.append(image_data)
    images_data = np.asarray(images_data).astype(np.float32)


    # Load TensorFlow Lite Models and allocate tensors.
    interpreter = tf.lite.Interpreter(model_path="/home/think/ros_project/catkin_ws/src/raspi_cam/yolov4_TFLite_model/yolov4_full_int8_tiny.tflite")
    interpreter.allocate_tensors()

    # Get input and output tensors.
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()

    # Perform the actual detection by running the model with the image as input
    # Sets the value of the input tensor.
    interpreter.set_tensor(input_details[0]['index'], images_data)

    # Invoke the interpreter.
    # Be sure to set the input sizes, allocate tensors and fill values before calling this.
    interpreter.invoke()

    # Retrieve detection results
    pred = [interpreter.get_tensor(output_details[i]['index']) for i in range(len(output_details))]

    boxes, pred_conf = filter_boxes(pred[0], pred[1], score_threshold=0.25, input_shape=tf.constant([input_size, input_size]))

    # Apply non-max suppression
    boxes, scores, classes, valid_detections = tf.image.combined_non_max_suppression(
        boxes=tf.reshape(boxes, (tf.shape(boxes)[0], -1, 1, 4)),
        scores=tf.reshape(pred_conf, (tf.shape(pred_conf)[0], -1, tf.shape(pred_conf)[-1])),
        max_output_size_per_class=50,
        max_total_size=50,
        iou_threshold=iou_threshold,
        score_threshold=score_threshold
    )

    pred_bbox = [boxes.numpy(), scores.numpy(), classes.numpy(), valid_detections.numpy()]

    # Draw bounding boxes on the image
    class_name_path = "/home/think/ros_project/catkin_ws/src/raspi_cam/yolov4_TFLite_model/coco.names"
    classes = read_class_names(class_name_path)
    image = draw_bbox(frame, pred_bbox, classes)

    # image = Image.fromarray(image.astype(np.uint8))
    # image = cv2.cvtColor(np.array(image), cv2.COLOR_BGR2RGB)

    return image


def filter_boxes(box_xywh, scores, score_threshold=0.4, input_shape = tf.constant([416,416])):
    """
    Filters YOLO boxes by thresholding on socres
    """
    scores_max = tf.math.reduce_max(scores, axis=-1)

    mask = scores_max >= score_threshold
    class_boxes = tf.boolean_mask(box_xywh, mask)
    pred_conf = tf.boolean_mask(scores, mask)
    class_boxes = tf.reshape(class_boxes, [tf.shape(scores)[0], -1, tf.shape(class_boxes)[-1]])
    pred_conf = tf.reshape(pred_conf, [tf.shape(scores)[0], -1, tf.shape(pred_conf)[-1]])

    box_xy, box_wh = tf.split(class_boxes, (2, 2), axis=-1)

    input_shape = tf.cast(input_shape, dtype=tf.float32)

    box_yx = box_xy[..., ::-1]
    box_hw = box_wh[..., ::-1]

    box_mins = (box_yx - (box_hw / 2.)) / input_shape
    box_maxes = (box_yx + (box_hw / 2.)) / input_shape
    boxes = tf.concat([
        box_mins[..., 0:1],  # y_min
        box_mins[..., 1:2],  # x_min
        box_maxes[..., 0:1],  # y_max
        box_maxes[..., 1:2]  # x_max
    ], axis=-1)
    # return tf.concat([boxes, pred_conf], axis=-1)
    return (boxes, pred_conf)

def draw_bbox(image, bboxes, classes, show_label=True):
    num_classes = len(classes)
    image_h, image_w, _ = image.shape
    hsv_tuples = [(1.0 * x / num_classes, 1., 1.) for x in range(num_classes)]
    colors = list(map(lambda x: colorsys.hsv_to_rgb(*x), hsv_tuples))
    colors = list(map(lambda x: (int(x[0] * 255), int(x[1] * 255), int(x[2] * 255)), colors))

    random.seed(0)
    random.shuffle(colors)
    random.seed(None)

    out_boxes, out_scores, out_classes, num_boxes = bboxes
    for i in range(num_boxes[0]):
        if int(out_classes[0][i]) < 0 or int(out_classes[0][i]) > num_classes: continue
        coor = out_boxes[0][i]
        coor[0] = int(coor[0] * image_h)
        coor[2] = int(coor[2] * image_h)
        coor[1] = int(coor[1] * image_w)
        coor[3] = int(coor[3] * image_w)

        fontScale = 0.5
        score = out_scores[0][i]
        class_ind = int(out_classes[0][i])
        bbox_color = colors[class_ind]
        bbox_thick = int(0.6 * (image_h + image_w) / 600)
        c1, c2 = (coor[1], coor[0]), (coor[3], coor[2])
        cv2.rectangle(image, c1, c2, bbox_color, bbox_thick)

        if show_label:
            bbox_mess = '%s: %.2f' % (classes[class_ind], score)
            t_size = cv2.getTextSize(bbox_mess, 0, fontScale, thickness=bbox_thick // 2)[0]
            c3 = (c1[0] + t_size[0], c1[1] - t_size[1] - 3)
            cv2.rectangle(image, c1, (np.float32(c3[0]), np.float32(c3[1])), bbox_color, -1) #filled

            cv2.putText(image, bbox_mess, (c1[0], np.float32(c1[1] - 2)), cv2.FONT_HERSHEY_SIMPLEX,
                        fontScale, (0, 0, 0), bbox_thick // 2, lineType=cv2.LINE_AA)
    return image

def read_class_names(class_file_name):
    names = {}
    with open(class_file_name, 'r') as data:
        for ID, name in enumerate(data):
            names[ID] = name.strip('\n')
    return names

if __name__ == '__main__':
    try:
        CameraSubscriber()
    except rospy.ROSInterruptException:
        pass