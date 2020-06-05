#! /usr/bin/python2


from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
bridge = CvBridge()

import cv2
import glob
import matplotlib.pyplot as plt
import pickle
import matplotlib.image as mpimg
import rospy
import time
import sys

cv2_img = None

import numpy as np
import os
import pathlib
import six.moves.urllib as urllib
import sys
import tarfile
import tensorflow as tf
import zipfile

from collections import defaultdict
from io import StringIO
from matplotlib import pyplot as plt
# from PIL import Image
# from IPython.display import display

from object_detection.utils import ops as utils_ops
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util


# Import everything needed to edit/save/watch video clips
# from moviepy.editor import VideoFileClip
# from IPython.display import HTML

# patch tf1 into `utils.ops`
utils_ops.tf = tf.compat.v1
# Patch the location of gfile
tf.gfile = tf.io.gfile

# List of the strings that is used to add correct label for each box.
# PATH_TO_LABELS = '/home/ruijie/Ruijie/Machine_Learning_Framework/TensorFlow-models/models/research/object_detection/data/mscoco_label_map.pbtxt'
PATH_TO_LABELS = '/home/ruijie/Ruijie/Machine_Learning_Framework/TensorFlow-models/workspace/training_demo/annotations/label_map.pbtxt'
## full path
category_index = label_map_util.create_category_index_from_labelmap(PATH_TO_LABELS, use_display_name=True)

def load_model(model_dir):
    model_dir = pathlib.Path(model_dir)/"saved_model"
    model = tf.saved_model.load(str(model_dir))
    model = model.signatures['serving_default']
    return model


model_dir = '/home/ruijie/Ruijie/Machine_Learning_Framework/TensorFlow-models/workspace/training_demo/trained-inference-graphs/output_inference_graph_v3.pb/'
detection_model = load_model(model_dir)

def show_inference(model, image_np):
    # the array based representation of the image will be used later in order to prepare the
    # result image with boxes and labels on it.
    # image_np = np.array(Image.open(image_path))
    # Actual detection.
    output_dict = run_inference_for_single_image(model, image_np)
    # Visualization of the results of a detection.
    vis_util.visualize_boxes_and_labels_on_image_array(
        image_np,
        output_dict['detection_boxes'],
        output_dict['detection_classes'],
        output_dict['detection_scores'],
        category_index,
        instance_masks=output_dict.get('detection_masks_reframed', None),
        use_normalized_coordinates=True,
        line_thickness=8
    )
    cv2.imshow('img', image_np)
    cv2.waitKey(1)

    # display(Image.fromarray(image_np))

def run_inference_for_single_image(model, image):
    image = np.asarray(image)
    # The input needs to be a tensor, convert it using `tf.convert_to_tensor`.
    input_tensor = tf.convert_to_tensor(image)
    # The model expects a batch of images, so add an axis with `tf.newaxis`.
    input_tensor = input_tensor[tf.newaxis, ...]

    # Run inference
    output_dict = model(input_tensor)

    # All outputs are batches tensors.
    # Convert to numpy arrays, and take index [0] to remove the batch dimension.
    # We're only interested in the first num_detections.
    num_detections = int(output_dict.pop('num_detections'))
    output_dict = {key: value[0, :num_detections].numpy()
                   for key, value in output_dict.items()}
    output_dict['num_detections'] = num_detections

    # detection_classes should be ints.
    output_dict['detection_classes'] = output_dict['detection_classes'].astype(np.int64)

    # Handle models with masks:
    if 'detection_masks' in output_dict:
        # Reframe the the bbox mask to the image size.
        detection_masks_reframed = utils_ops.reframe_box_masks_to_image_masks(
            output_dict['detection_masks'], output_dict['detection_boxes'],
            image.shape[0], image.shape[1])
        detection_masks_reframed = tf.cast(detection_masks_reframed > 0.5,
                                           tf.uint8)
        output_dict['detection_masks_reframed'] = detection_masks_reframed.numpy()

    return output_dict


def imshow(img, title=None, as_float=False, scale=None):
    # Quick imshow helper function for debugging
    if scale is not None:
        img = cv2.resize(img, (0,0), fx=scale, fy=scale)
    if not as_float:
        img = img.astype(np.uint8)
    if title is not None:
        cv2.imshow(title, img)
    else:
        cv2.imshow('Quick imshow', img)
    #cv2.waitKey(0)
    cv2.destroyAllWindows()

def image_callback(msg):
    print("Received an image!")
    global cv2_img
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")

        # load the model
        show_inference(detection_model, cv2_img)
        # result = run_inference_for_single_image(detection_model, cv2_img)

    except CvBridgeError as e:
        pass
    else:
        # Save your OpenCV2 image as a jpeg
        # cv2.imwrite('camera_image.jpeg', cv2_img)
        # process_img(cv2_img)
        pass


def main():
    rospy.init_node('image_listener')
    # Define your image topic
    image_topic = "/camera/color/image_raw"

    # Set up your subscriber and define its callback
    print(1)
    rospy.Subscriber(image_topic, Image, image_callback)
    # time.sleep(3)
    # Spin until ctrl + c
    model_dir = '/home/ruijie/Ruijie/Machine_Learning_Framework/TensorFlow-models/workspace/training_demo/trained-inference-graphs/output_inference_graph_v3.pb/'
    detection_model = load_model(model_dir)

    rospy.spin()
    # while 1:
    #     if cv2_img is not None:
    #         pass
    #         # process_img(cv2_img)
    #         # cv2.imshow('img', cv2_img)
    #
    #     if cv2.waitKey(1) & 0xFF == ord('q'):
    #         break
    #
    #     time.sleep(.2)
    # cv2.destroyAllWindows()
    # rospy.spin()

    # img = cv2.imread('test3.jpeg',cv2.IMREAD_COLOR)
    # process_img(img)


if __name__ == '__main__':

    # get image from ros topic
    main()

    # get test image from local.
    # '''
    # image = cv2.imread("outdoor_lanes.png")
    # process_img(image)
    # '''

    """
    load the tensorflow model
    """
    # model_dir = '/home/ruijie/Ruijie/Machine_Learning_Framework/TensorFlow-models/workspace/training_demo/trained-inference-graphs/output_inference_graph_v3.pb/'
    # detection_model = load_model(model_dir)
