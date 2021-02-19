#! /usr/bin/python2
import message_filters
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
import argparse

import numpy as np
import os
# import pathlib
import six.moves.urllib as urllib
import sys
import tarfile
import tensorflow as tf
import zipfile

import cob_object_detection_msgs.msg._DetectionArray
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

cv2_img = None
depth_img = None

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
    # cv2.waitKey(0)
    cv2.destroyAllWindows()

def load_model(model_dir):
    model_dir = model_dir + "/saved_model"
    # model_dir = pathlib.Path(model_dir)/"saved_model"
    # print model_dir
    model = tf.saved_model.load(str(model_dir))
    model = model.signatures['serving_default']
    return model


# model_dir = '/home/ruijie/Ruijie/Machine_Learning_Framework/TensorFlow-models/workspace/training_demo/trained-inference-graphs/output_inference_graph_v3.pb/'
# detection_model = load_model(model_dir)

def publish_detection(output_dict, num_detections, shape):
    """
    :param output_dict:
    :param num_detections:
    :param shape: (height, width, channel)
    :return:
    """
    global depth_img
    detection_objects = cob_object_detection_msgs.msg.DetectionArray()
    for key, item in category_index.items():
        object = cob_object_detection_msgs.msg.Detection()
        object.id = item["id"]
        object.label = item["name"]
        object.score = 0
        detection_objects.detections.append(object)


    for i in range(num_detections):
        for object in detection_objects.detections:
            if (object.id == output_dict['detection_classes'][i] and
                    object.score < output_dict['detection_scores'][i]):
                object.score = output_dict['detection_scores'][i]
                bbox = list(output_dict['detection_boxes'][i])
                bbox[0] = int(bbox[0]*shape[0])
                bbox[1] = int(bbox[1]*shape[1])
                bbox[2] = int(bbox[2]*shape[0])
                bbox[3] = int(bbox[3]*shape[1])
                ## debug
                # sub_depth = depth_img[bbox[1]:bbox[3], bbox[0]:bbox[2]]
                # print sub_depth
                # np.set_printoptions(threshold=sys.maxsize)
                # print depth_img[bbox[1]:bbox[3], bbox[0]:]
                ## debug
                # object.depth = np.median(sub_depth)
                # print "bbox is {}".format(bbox)
                # print "depth is {}, name is {}".format(object.depth, object.label)
                # sub_depth = np.reshape(sub_depth, sub_depth.shape[0] * sub_depth.shape[1])
                # print sub_depth.shape
                # print sub_depth
                # sub_depth = sub_depth[np.where((sub_depth > 100) & (sub_depth < 400))]
                # print "new depth is {}".format(np.mean(sub_depth))

                object.mask.roi.x = bbox[1]
                object.mask.roi.y = bbox[0]
                object.mask.roi.width = (bbox[3]-bbox[1])
                object.mask.roi.height = (bbox[2]-bbox[0])

    pub_detected_object.publish(detection_objects)




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
        line_thickness=8,
        min_score_thresh=0.1
    )
    cv2.imshow('img', image_np)
    cv2.imwrite("2020-02-11-demo.jpg", image_np)
    cv2.waitKey(1)
    print os.getcwd()
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

    publish_detection(output_dict, num_detections, image.shape)

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



def image_callback(color_img_msg, depth_img_msg):
    print("Received an image!")
    global cv2_img
    global depth_img
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(color_img_msg, "bgr8")
        depth_img = bridge.imgmsg_to_cv2(depth_img_msg, "32FC1")

        # load the model
        # result = run_inference_for_single_image(detection_model, cv2_img)
        result = show_inference(detection_model, cv2_img)

        # cv2.imwrite('camera_image.jpg', cv2_img)
        # sys.exit(0)

        # print len(result)
        # pub("bottle", [10, 10, 50, 100], 100)
        # result = run_inference_for_single_image(detection_model, cv2_img)

    except CvBridgeError as e:
        pass
    else:
        # Save your OpenCV2 image as a jpeg
        # cv2.imwrite('camera_image.jpeg', cv2_img)
        # process_img(cv2_img)
        pass

def single_image_callback(color_img_msg):
    print("Received an image!")
    global cv2_img
    # try:
    # Convert your ROS Image message to OpenCV2
    cv2_img = bridge.imgmsg_to_cv2(color_img_msg, "bgr8")
    result = show_inference(detection_model, cv2_img)
    cv2.imwrite('camera_image123.jpg'.format(time.time()), cv2_img)
    # print "image writed"

    # except CvBridgeError as e:
    #     pass
    # else:
    #     pass



if __name__ == '__main__':
    rospy.init_node('object_detection_node')

    parser = argparse.ArgumentParser(
        description="Use tensorflow model to do object detection")
    parser.add_argument("-m",
                        "--modelDir",
                        help="Path to the folder where the model files are stored",
                        type=str)
    parser.add_argument("-l",
                        "--labelFile",
                        help="Name of label file (including path)", type=str)
    args = parser.parse_args(sys.argv[1:5])

    print args.modelDir
    # print args.labelFile

    # Publisher
    pub_detected_object = rospy.Publisher("/detected_object", cob_object_detection_msgs.msg.DetectionArray, queue_size=1)

    # model_dir = '/home/ruijie/Ruijie/Machine_Learning_Framework/TensorFlow-models/workspace/training_demo/trained-inference-graphs/output_inference_graph_v3.pb/'
    model_dir = args.modelDir
    detection_model = load_model(model_dir)

    # List of the strings that is used to add correct label for each box.
    # PATH_TO_LABELS = '/home/ruijie/Ruijie/Machine_Learning_Framework/TensorFlow-models/workspace/training_demo/annotations/label_map.pbtxt'
    PATH_TO_LABELS = args.labelFile
    ## full path
    category_index = label_map_util.create_category_index_from_labelmap(PATH_TO_LABELS, use_display_name=True)

    # Subscriber
    rospy.Subscriber("/camera/color/image_raw", Image, single_image_callback)
    # image_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
    # depth_sub = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image)
    # ts = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub], 10, 2)
    # ts.registerCallback(image_callback)

    rospy.spin()

