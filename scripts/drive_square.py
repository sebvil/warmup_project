#!/usr/bin/env python3

import math

import rospy
from geometry_msgs.msg import Twist


class SquareDriver:
    def __init__(self):
        rospy.init_node("square_driver")
        self.publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/cmd_vel", Twist, self._process_speed)

    def _process_speed(self, data):
        print(data)

    def _drive_in_straight_line(self):
        forward_time = 9
        target_velocity = 0.3
        acceleration_frequency = (
            60  # rate (in Hz) at which robot will accelerate
        )
        acceleration = target_velocity / (
            acceleration_frequency * forward_time / 3
        )
        current_velocity = 0

        speed = Twist()
        rate = rospy.Rate(acceleration_frequency)

        # Accelartion phase
        while current_velocity < target_velocity:
            speed.linear.x = current_velocity
            self.publisher.publish(speed)
            current_velocity += acceleration
            rate.sleep()

        # Constant speed phase
        t0 = rospy.Time.now().to_sec()
        t1 = t0
        while t1 - t0 < forward_time / 3:
            self.publisher.publish(speed)
            t1 = rospy.Time.now().to_sec()
            rate.sleep()

        # Slow down phase
        while current_velocity > 0:
            speed.linear.x = current_velocity
            self.publisher.publish(speed)
            current_velocity -= acceleration
            rate.sleep()

        # Ensure velocity is 0
        speed.linear.x = 0
        self.publisher.publish(speed)

    def _turn(self):
        turn_time = 7
        target_angle = math.pi / 2
        acceleration_frequency = (
            60  # rate (in Hz) at which robot will accelerate
        )
        rate = rospy.Rate(acceleration_frequency)
        acceleration = (
            4 * target_angle / (acceleration_frequency * (turn_time ** 2))
        )

        current_velocity = 0
        speed = Twist()
        speed.angular.z = current_velocity

        t0 = rospy.Time.now().to_sec()
        t1 = t0

        while t1 - t0 < turn_time / 2:
            self.publisher.publish(speed)
            current_velocity += acceleration
            speed.angular.z = current_velocity

            t1 = rospy.Time.now().to_sec()
            rate.sleep()

        while t1 - t0 < turn_time:
            self.publisher.publish(speed)
            current_velocity -= acceleration
            speed.angular.z = current_velocity

            t1 = rospy.Time.now().to_sec()
            rate.sleep()

        speed.angular.z = 0
        self.publisher.publish(speed)

    def _do_one_loop(self):
        self._drive_in_straight_line()
        rospy.sleep(1)  # make sure robot has stopped before beginning turn.
        self._turn()
        rospy.sleep(1)  # make sure robot has stopped moving again.

    def run(self):

        while not rospy.is_shutdown():
            self._do_one_loop()


if __name__ == "__main__":
    SquareDriver().run()
