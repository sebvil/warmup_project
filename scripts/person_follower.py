#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class PersonFollower:
    def __init__(self):
        rospy.init_node("person_follower")
        self.publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.speed = Twist()
        rospy.Subscriber("/cmd_vel", Twist, self._process_speed)
        rospy.Subscriber("/scan", LaserScan, self._proccess_scan)

    def _process_speed(self, data):
        pass
        # print(data)

    def _proccess_scan(self, data):
        min_dist = min(data.ranges)
        min_angle = data.ranges.index(min_dist)
        print(min_angle, min_dist)
        if 20 < min_angle <= 180:
            self.speed.angular.z = 1
        elif 180 < min_angle < 340:
            self.speed.angular.z = -1
        else:
            self.speed.angular.z = 0

        if min_dist > 1 and min_dist != float("inf"):
            self.speed.linear.x = 0.5
        else:
            self.speed.linear.x = 0

        self.publisher.publish(self.speed)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    PersonFollower().run()
