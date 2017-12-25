from styx_msgs.msg import TrafficLight
import rospy
import tensorflow as tf
import numpy as np
from PIL import Image

import os

if tf.__version__ < '1.4.0':
    raise ImportError('Please upgrade your tensorflow installation to v1.4.* or later!')

from utils import label_map_util

# from utils import visualization_utils as vis_util


class TLClassifier(object):

    # What model to download.
    MODEL_NAME = 'light_detector_model'

    # Path to frozen detection graph. This is the actual model that is used for the object detection.
    PATH_TO_CKPT = MODEL_NAME + '/frozen_inference_graph.pb'

    # List of the strings that is used to add correct label for each box.
    PATH_TO_LABELS = os.path.join('model_config', 'light_label_map.pbtxt')

    NUM_CLASSES = 3
    label_map = None
    categories = None
    category_index = None

    detection_graph = None

    # Definite input and output Tensors for detection_graph
    image_tensor_t = None
    # Each box represents a part of the image where a particular object was detected.
    detection_boxes_t = None
    # Each score represent how level of confidence for each of the objects.
    # Score is shown on the result image, together with the class label.
    detection_scores_t = None
    detection_classes_t = None
    num_detections_t = None

    def __init__(self, path):
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(path or self.PATH_TO_CKPT, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

        self.label_map = label_map_util.load_labelmap(self.PATH_TO_LABELS)
        self.categories = label_map_util.convert_label_map_to_categories(
            self.label_map, max_num_classes=self.NUM_CLASSES, use_display_name=True)
        self.category_index = label_map_util.create_category_index(self.categories)

        with self.detection_graph.as_default():
            with tf.Session(graph=self.detection_graph) as sess:
                # Definite input and output Tensors for detection_graph
                self.image_tensor_t = self.detection_graph.get_tensor_by_name('image_tensor:0')
                # Each box represents a part of the image where a particular object was detected.
                self.detection_boxes_t = self.detection_graph.get_tensor_by_name(
                    'detection_boxes:0')
                # Each score represent how level of confidence for each of the objects.
                # Score is shown on the result image, together with the class label.
                self.detection_scores_t = self.detection_graph.get_tensor_by_name(
                    'detection_scores:0')
                self.detection_classes_t = self.detection_graph.get_tensor_by_name(
                    'detection_classes:0')
                self.num_detections_t = self.detection_graph.get_tensor_by_name('num_detections:0')

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
        image_np_expanded = np.expand_dims(image , axis=0)

        with self.detection_graph.as_default():
            with tf.Session(graph=self.detection_graph) as sess:
                boxes, scores, classes, num = sess.run(
                        [self.detection_boxes_t, self.detection_scores_t, self.detection_classes_t, self.num_detections_t],
                        feed_dict={self.image_tensor_t: image_np_expanded})

        output = 0
        candidate_num = 4
        vote = []
        scorestemp =scores[0]
        classestemp=classes[0]
        for i in range(candidate_num):
            if scorestemp[i] < 0.5:
                break
            vote.append(self.__label_map_to_traffic_light(int(classestemp[i])))
        if vote:
            return max(vote, key=vote.count)
        else:
            return 4

        return output

    def __label_map_to_traffic_light(self, label_id):
        traffic_label = int(label_id) - 1
        if traffic_label in [0, 1, 2, 4]:
            return  traffic_label
        return 4

