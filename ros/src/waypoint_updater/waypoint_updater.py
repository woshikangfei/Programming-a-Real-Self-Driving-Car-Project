#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint

import math
from std_msgs.msg import Int32, Header
import tf


'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200  # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        self.waypoint = None
        self.current_pose = None

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        self.traffic_waypoint = None
        self.obstacle_waypoint = None
        self.velocity = None

        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)
        rospy.Subscriber('/current_velocity', TwistStamped,
                         self.current_velocity_cb)

        self.final_waypoints_pub = rospy.Publisher(
            '/final_waypoints', Lane, queue_size=1)

        rate_limiter = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.loop()
            rate_limiter.sleep()

    def pose_cb(self, msg):
        self.current_pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoint = waypoints

    def traffic_cb(self, msg):
        self.traffic_waypoint = msg.data

    def current_velocity_cb(self, msg):
        self.velocity = msg.twist.linear.x

    def obstacle_cb(self, msg):
        self.obstacle_waypoint = msg.data

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0

        for i in range(wp1, wp2 + 1):
            dist += self.dist(waypoints[wp1].pose.pose.position,
                       waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def loop(self):
        if self.waypoint and self.current_pose:
            index = self.get_waypoint(self.current_pose.pose)
            nindex = self.get_nwaypint(self.current_pose.pose, index)
            lane = Lane()
            lane.header.frame_id = '/world'
            lane.header.stamp = rospy.Time(0)

            if self.velocity is not None:
                min_dist_stop = self.velocity**2 / (2.0 * 5.0) + 32.0
            else:
                min_dist_stop = 32.0

            if self.traffic_waypoint and self.traffic_waypoint != -1:
                tl_dist = self.distance(
                    self.waypoint.waypoints, index, self.traffic_waypoint)

                rospy.loginfo("self.velocity {}, self.traffic_waypoint {}, tl_dist {}, min_dist_stop {}".format(
                    self.velocity,
                    self.traffic_waypoint,
                    tl_dist,
                    min_dist_stop))

                if tl_dist < min_dist_stop:
                    rospy.loginfo("braking")
                    final_waypoints = []
                    for i in range(nindex, self.traffic_waypoint):
                        index = i % len(self.waypoint.waypoints)
                        wp = Waypoint()
                        wp.pose.pose.position.x = self.waypoint.waypoints[index].pose.pose.position.x
                        wp.pose.pose.position.y = self.waypoint.waypoints[index].pose.pose.position.y
                        wp.pose.pose.position.z = self.waypoint.waypoints[index].pose.pose.position.z
                        wp.pose.pose.orientation = self.waypoint.waypoints[index].pose.pose.orientation
                        final_waypoints.append(wp)
                    # set speed to stop before traffic light
                    if len(final_waypoints) != 0:
                        lane.waypoints = self.getWaypoints(final_waypoints)
                else:
                    rospy.loginfo("no braking  traffic light")
                    lane.waypoints = self.waypoint.waypoints[nindex : (nindex + LOOKAHEAD_WPS )]

            else:
                rospy.loginfo("no   traffic light  ")
                lane.waypoints = self.waypoint.waypoints[nindex : (nindex + LOOKAHEAD_WPS )]

            self.final_waypoints_pub.publish(lane)

    def getWaypoints(self, waypoints):
        last = waypoints[-1]
        last.twist.twist.linear.x = 0.0
        for wp in waypoints[:-1][::-1]:
            dist = self.dist(wp.pose.pose.position,
                             last.pose.pose.position)
            dist = max(0.0, dist - 32.0)
            velsppend = math.sqrt(2 * 5.0 * dist)
            if velsppend < 1.0:
                velsppend = 0.0
            wp.twist.twist.linear.x = min(velsppend, wp.twist.twist.linear.x)
        return waypoints

    def dist(self, p1, p2):
        x, y, z = p1.x - p2.x, p1.y - p2.y, p1.z - p2.z
        return math.sqrt(x * x + y * y + z * z)

    def get_waypoint(self, pose):
        closest_index = 0
        closest_dist = 100000
        p1 = pose.position
        wl = self.waypoint.waypoints

        for i in range(len(wl)):
            p2 = wl[i].pose.pose.position
            d = self.dist(p1, p2)
            if(d < closest_dist):
                closest_dist = d
                closest_index = i

        return closest_index

    def get_nwaypint(self, pose, index):
        nindex = index
        p1 = pose.position
        p2 = self.waypoint.waypoints[index].pose.pose.position
        heading = math.atan2((p2.y - p1.y), (p2.x - p1.x))
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
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
