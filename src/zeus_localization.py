#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from threading import Lock
from copy import deepcopy

class ZeusLocalization():
    def __init__(self):
        rospy.init_node("zeus_localization", anonymous = True)
        self.mutex = Lock()

        self.prev_position = None
        self.latest_position = None
        self.heading_at_prev_position = None
        self.heading_at_latest_position = None

        self.prev_heading = None
        self.latest_heading = None
        self.latest_heading_timestamp = None

        self.world_frame = rospy.get_param("~world_frame", "map")

        self.localization_pub = rospy.Publisher('/localization_odom', Odometry, queue_size=1)
        self.position_sub = rospy.Subscriber('/gnss_map_pose', PoseWithCovarianceStamped, self.positionCB, queue_size=1)
        self.heading_sub = rospy.Subscriber('/gps_heading', Imu, self.headingCB, queue_size=1)

    def positionCB(self, msg):
        self.mutex.acquire()
        try:
            print("Got position")
            self.prev_position = deepcopy(self.latest_position)
            self.heading_at_prev_position = deepcopy(self.heading_at_latest_position)
            self.heading_at_latest_position = deepcopy(self.latest_heading)
            self.latest_position = deepcopy(msg)
            self.publishLocalization()
        finally:
            self.mutex.release()

    def headingCB(self, msg):
        self.mutex.acquire()
        try:
            print("Got heading")
            self.prev_heading = deepcopy(self.latest_heading)
            self.latest_heading = deepcopy(msg)
            self.latest_heading_timestamp = rospy.Time.now()
            self.publishLocalization()
        finally:
            self.mutex.release()

    def publishLocalization(self):
        print("publishing")
        if self.latest_position != None and self.latest_heading != None:
            print("publishing conditions met")
            msg = Odometry()
            msg.header.stamp = rospy.Time.now()
            msg.child_frame_id = 'map'
            msg.pose.pose.position.x = self.latest_position.pose.pose.position.x
            msg.pose.pose.position.y = self.latest_position.pose.pose.position.y
            msg.pose.pose.position.z = self.latest_position.pose.pose.position.z
            msg.pose.pose.orientation.x = self.latest_heading.orientation.x
            msg.pose.pose.orientation.y = self.latest_heading.orientation.y
            msg.pose.pose.orientation.z = self.latest_heading.orientation.z
            msg.pose.pose.orientation.w = self.latest_heading.orientation.w
            self.localization_pub.publish(msg)

    def calculatePositionTwist(self):
        if self.prev_position != None and self.latest_position != None and \
        self.heading_at_prev_position != None:
            # TODO
            pass


if __name__ == '__main__':
    zeus_localization = ZeusLocalization()
    rospy.loginfo("zeus_localization ready")
    rospy.spin()