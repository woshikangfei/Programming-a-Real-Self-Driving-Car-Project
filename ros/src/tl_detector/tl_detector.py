#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose, PointStamped, Point
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math

STATE_COUNT_THRESHOLD = 2

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights',
                                TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher(
            '/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.model = rospy.get_param('~model_path', None)
        if self.model == None or self.model == "None":
            self.light_classifier = None
        else:
            self.light_classifier = TLClassifier(self.model)

        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        self.camera_image_pub = rospy.Publisher(
            '/image_color_info', Image, queue_size=1)

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        # TODO implement
        closest_index = 0
        closest_dist = 100000.
        p1 = pose.position

        for index, waypoint in enumerate(self.waypoints.waypoints):
            p2 = waypoint.pose.pose.position
            d = self.dist(p1, p2)
            if d < closest_dist:
                closest_dist = d
                closest_index = index

        return closest_index

    def dist(self, x1, x2):
        x, y, z = x1.x - x2.x, x1.y - x2.y, x1.z - x2.z
        return math.sqrt(x * x + y * y + z * z)

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if not self.has_image:
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        # Get classification
        if self.light_classifier:
            return self.light_classifier.get_classification(cv_image)
        else:
            return TrafficLight.UNKNOWN

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

        if not self.pose:
            rospy.logwarn('no self.pose')
            return -1, TrafficLight.UNKNOWN

        if not self.waypoints:
            rospy.logwarn('no self.waypoints')
            return -1, TrafficLight.UNKNOWN

        stop_line_positions = self.config['stop_line_positions']
        car_position = self.get_closest_waypoint(self.pose.pose)

        closest_light_index = self.get_closest_trafficlight(
            self.pose.pose, stop_line_positions)
        next_light_index = self.get_next_trafficlight(
            self.pose.pose, closest_light_index, stop_line_positions)
        # loop for self.lights
        next_light_index = next_light_index % len(self.lights)
        light = self.lights[next_light_index]
        light_wp = self.get_closest_waypoint(light.pose.pose)

        if light:
            state = self.get_light_state(light)
            return light_wp, state

        return -1, TrafficLight.UNKNOWN

    def get_closest_trafficlight(self, pose, light_list):
        closest_index = 0
        closest_dist = 100000.
        p1 = pose.position

        for i in range(len(light_list)):
            p2 = light_list[i]
            x, y = p1.x - p2[0], p1.y - p2[1]
            d = math.sqrt(x * x + y * y)
            if d < closest_dist:
                closest_dist = d
                closest_index = i

        return closest_index

    def get_next_trafficlight(self, pose, index, light_list):
        nindex = index
        p1 = pose.position
        p2 = light_list[index]
        heading = math.atan2((p2[1] - p1.y), (p2[0] - p1.x))
        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]
        angle = abs(yaw - heading)

        if angle > math.pi / 4:
            nindex += 1

        return nindex


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
