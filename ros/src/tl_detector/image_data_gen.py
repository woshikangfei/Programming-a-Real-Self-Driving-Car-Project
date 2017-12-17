#!/usr/bin/env python
import os
import errno
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import cv2
import numpy as np


def mkdir_path(path):
    try:
        os.makedirs(path, exist_ok=True)  # Python>3.2
    except TypeError:
        try:
            os.makedirs(path)
        except OSError as exc:  # Python >2.5
            if exc.errno == errno.EEXIST and os.path.isdir(path):
                pass
            else:
                raise


class ImageDataGenerate(object):
    def __init__(self):
        rospy.init_node('image_data_generate')

        rospy.Subscriber('/image_raw', Image, self.image_callback)
        self.bridge = CvBridge()

        self.data_dir = "./data/"
        self.image_name = 0
        mkdir_path(self.data_dir)
        rospy.spin()

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            row, col, color = cv_image.shape
            cv_image = cv_image[int(0.2*col):int(0.5*col), 0:row]
            cv_image = self.adjust_gamma(cv_image, 0.7)
            cv2.imshow("Image window", cv_image)
            cv2.imwrite(self.data_dir + "data" + str(self.image_name) + ".png", cv_image)
            self.image_name += 1
            cv2.waitKey(3)
        except CvBridgeError as e:
            print(e)
        pass

    def adjust_gamma(self, image, gamma=1.0):
        inv_gamma = 1.0 / gamma
        table = np.array((np.arange(0, 256) / 255.0) ** inv_gamma * 255, dtype="uint8")
        return cv2.LUT(image, table)


if __name__ == '__main__':
    try:
        ImageDataGenerate()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start data generate node.')
