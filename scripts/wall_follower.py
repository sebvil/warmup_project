#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class WallFollower:
    def __init__(self):
        rospy.init_node("wall_follower")
        self.publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.speed = Twist()
        self.speed.linear.x = 0.3
        rospy.Subscriber("/scan", LaserScan, self._proccess_scan)
        self.found_wall = False

    def _proccess_scan(self, data):
        min_dist = min(data.ranges)
        min_angle = data.ranges.index(min_dist)

        if min_dist < 1:
            self.found_wall = True

        if not self.found_wall:
            self.publisher.publish(self.speed)
            return

        if min_angle > 280 or data.ranges[0] < 1:
            self.speed.angular.z = 0.6
        elif min_angle < 260:
            self.speed.angular.z = -0.6
        else:
            self.speed.angular.z = 0
        self.publisher.publish(self.speed)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    WallFollower().run()
