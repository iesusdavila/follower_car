#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import sys
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class FollowTB3Robotics(Node):
    def __init__(self):        
        argv = sys.argv
        if len(argv) < 2:
            print("It is required to specify the name of the TurtleBot3 as an argument to the executable. For example: car_follower")
            sys.exit(1)
        self.name_robot = argv[1]
        scan_topic = self.name_robot + "/scan"
        cmd_vel_topic = self.name_robot + "/cmd_vel"

        super().__init__(f'follow_{self.name_robot}_node')

        self.sub_scan = self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)
        self.pub_cmd_vel = self.create_publisher(Twist, cmd_vel_topic, 10)

        self.LIM_DISTANCE = 0.3
        self.LIM_ANGULAR_VELOCITY = 0.2
        self.LIM_LINEAR_VELOCITY = 0.22
        self.GAIN_Kp = 1.0

        self.scanned_init_angle = 60
        self.scanned_fin_angle = 120

    def limit_velocity(self, vel, lim_vel):
        if vel > lim_vel:
            vel = lim_vel
        elif vel < -lim_vel:
            vel = -lim_vel
        return float(vel)
    
    def filter_inf(self, ranges, start=None, end=None):
        if start is None and end is None:
            return [val for val in ranges if not math.isinf(val)]
        else:
            return [val for val in ranges[start:end] if not math.isinf(val)]    

    def get_quadrants(self):
        # Fourth Quadrant
        is_foq_ia = self.scanned_init_angle < 360 and self.scanned_init_angle > 270 # Init angle
        is_foq_fa = self.scanned_fin_angle < 360 and self.scanned_fin_angle > 270 # Final angle
        # First Quadrant
        is_fiq_ia = self.scanned_init_angle > 0 and self.scanned_init_angle < 90 # Init angle
        is_fiq_fa = self.scanned_fin_angle > 0 and self.scanned_fin_angle < 90 # Final angle

        return is_foq_ia, is_foq_fa, is_fiq_ia, is_fiq_fa

    def scan_car_ranges(self, ranges):
        is_foq_ia, is_foq_fa, is_fiq_ia, is_fiq_fa = self.get_quadrants()

        scan_fwrd_ranges = []
        if is_fiq_ia and is_fiq_fa:
            scan_fwrd_ranges = ranges[self.scanned_init_angle:self.scanned_fin_angle]
        elif is_foq_ia and is_foq_fa:
            scan_fwrd_ranges = ranges[self.scanned_init_angle:self.scanned_fin_angle]
        elif is_foq_ia and is_fiq_fa:
            scan_fwrd_ranges = ranges[:self.scanned_fin_angle] + ranges[self.scanned_init_angle:]
        else:
            scan_fwrd_ranges = ranges[self.scanned_init_angle:self.scanned_fin_angle]
        
        return self.filter_inf(scan_fwrd_ranges)

    def move_linear(self, ranges):
        fwrd_ranges = self.scan_car_ranges(ranges)

        vel_linear_x = 0.0

        if fwrd_ranges:
            error = min(fwrd_ranges) - self.LIM_DISTANCE
            linear_vel = self.GAIN_Kp * error # Control Proporcional
            
            vel_linear_x = self.limit_velocity(linear_vel, self.LIM_LINEAR_VELOCITY)

        return vel_linear_x

    def select_range_lim_front(self):
        is_foq_ia, is_foq_fa, is_fiq_ia, is_fiq_fa = self.get_quadrants()

        range_limit_front = 0.0
        if is_fiq_ia and is_fiq_fa:
            range_limit_front = self.scanned_fin_angle - self.scanned_init_angle
        elif is_foq_ia and is_foq_fa:
            range_limit_front = self.scanned_fin_angle - self.scanned_init_angle
        elif is_foq_ia and is_fiq_fa:
            range_limit_front = (360 - self.scanned_init_angle) + self.scanned_fin_angle
        else:
            range_limit_front = self.scanned_fin_angle - self.scanned_init_angle

        return range_limit_front

    def select_range_lim_left(self):
        is_foq_ia, is_foq_fa, is_fiq_ia, is_fiq_fa = self.get_quadrants()

        range_limit_left = 0.0
        if is_fiq_ia:
            range_limit_left = 90 - self.scanned_fin_angle
        elif is_foq_ia:
            range_limit_left = 90 + (360 - self.scanned_fin_angle)
        else:
            range_limit_left = 0

        return range_limit_left

    def select_range_lim_right(self):
        is_foq_ia, is_foq_fa, is_fiq_ia, is_fiq_fa = self.get_quadrants()

        range_limit_right = 0.0
        if is_fiq_fa:
            range_limit_right = 90 + self.scanned_init_angle
        elif is_foq_fa:
            range_limit_right = self.scanned_init_angle - 270
        else:
            range_limit_right = 0

        return range_limit_right

    def move_angular(self):
        range_limit_front = self.select_range_lim_front()
        range_limit_left = self.select_range_lim_left()
        range_limit_right = self.select_range_lim_right()
        
        angular_vel = 0.0

        if (range_limit_front<20) and (range_limit_left > range_limit_right):
            angular_vel = -self.LIM_ANGULAR_VELOCITY
        elif (range_limit_front<20) and (range_limit_left < range_limit_right):
            angular_vel = self.LIM_ANGULAR_VELOCITY

        return angular_vel

    def detect_tb3(self, ranges):
        init_angle = None
        fin_angle = None

        # Find the car from 345 to 15 degrees
        i = 345
        scan_ranges = ranges[345:] + ranges[:15]

        for scan in scan_ranges:
            if not math.isinf(scan):
                if scan < self.LIM_DISTANCE*2:
                    if init_angle is not None:
                        fin_angle = i
                    if init_angle is None:
                        init_angle = i
            if i == 359:
                i = 0
            else:
                i += 1

        val_detect_front = True

        if init_angle is not None and fin_angle is not None:
            self.info("Rover detected between ia: {}° and fa: {}°".format(init_angle, fin_angle))
            self.scanned_init_angle = init_angle
            self.scanned_fin_angle = fin_angle
        else:
            self.info("No car detected between ia: {}° and fa: {}°".format(345, 15))
            val_detect_front = False

        return val_detect_front         

    def scan_callback(self, data):
        ranges = data.ranges

        twist = Twist()

        scan_front = ranges[:90] + ranges[270:]
        if not (any(self.filter_inf(scan_front))):
            self.info("No car detected. Stopping the car.")
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.pub_cmd_vel.publish(twist)
            return

        is_detect_tb3 = self.detect_tb3(ranges)
        if is_detect_tb3:
            twist.linear.x = self.move_linear(ranges)
            twist.angular.z = self.move_angular()
            self.info(f'VelX: {twist.linear.x} VelZ: {twist.angular.z}')
            self.pub_cmd_vel.publish(twist)
    
    def info(self, msg):
        self.get_logger().info(msg)
        return

def main(args=None):
    rclpy.init(args=args)
    follow_tb3_robotics = FollowTB3Robotics()
    rclpy.spin(follow_tb3_robotics)
    follow_tb3_robotics.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
