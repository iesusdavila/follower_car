#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import sys
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class FollowTB3Burger(Node):
    def __init__(self): 
        argv = sys.argv
        if len(argv) >= 2:
            self.name_robot = argv[1]
        else:
            self.name_robot = ""

        super().__init__(f'follow_{self.name_robot}_node')
        self.info("Node started")

        scan_topic = self.name_robot + "/scan"
        cmd_vel_topic = self.name_robot + "/cmd_vel"

        self.sub_scan = self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)
        self.pub_cmd_vel = self.create_publisher(Twist, cmd_vel_topic, 10)

        self.LIM_DISTANCE = 0.3
        self.LIM_ANGULAR_VELOCITY = 0.2 #0.08
        self.LIM_LINEAR_VELOCITY = 0.22 #0.01
        self.GAIN_Kp = 3.0

        self.scanned_init_angle = 0
        self.scanned_fin_angle = 505

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
        # Second Quadrant
        is_sq_ia = self.scanned_init_angle >= 126 and self.scanned_init_angle <= 252 # Init angle
        is_sq_fa = self.scanned_fin_angle >= 126 and self.scanned_fin_angle <= 252 # Final angle

        # Third Quadrant
        is_tq_ia = self.scanned_init_angle > 252 and self.scanned_init_angle <= 378 # Init angle
        is_tq_fa = self.scanned_init_angle > 252 and self.scanned_init_angle <= 378 # Init angle

        return is_sq_ia, is_sq_fa, is_tq_ia, is_tq_fa

    def scan_car_ranges(self, ranges):
        is_sq_ia, is_sq_fa, is_tq_ia, is_tq_fa = self.get_quadrants()

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
        is_sq_ia, is_sq_fa, is_tq_ia, is_tq_fa = self.get_quadrants()

        range_limit_front = self.scanned_fin_angle - self.scanned_init_angle

        return range_limit_front

    def select_range_lim_left(self):
        is_sq_ia, is_sq_fa, is_tq_ia, is_tq_fa = self.get_quadrants()

        range_limit_left = 0.0
        if is_sq_ia:
            range_limit_left = 126 + (252 - self.scanned_fin_angle)
        elif is_tq_ia:
            range_limit_left = 378 - self.scanned_fin_angle
        else:
            range_limit_left = 0

        return range_limit_left

    def select_range_lim_right(self):
        is_sq_ia, is_sq_fa, is_tq_ia, is_tq_fa = self.get_quadrants()

        range_limit_right = 0.0
        if is_sq_fa:
            range_limit_right = self.scanned_init_angle - 126
        elif is_tq_fa:
            range_limit_right = 126 + (self.scanned_init_angle - 252)
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

        # Find the car from 237 to 267 degrees
        i = 237
        scan_ranges = ranges[237:267]

        for scan in scan_ranges:
            if not math.isinf(scan):
                if scan < self.LIM_DISTANCE*2:
                    if init_angle is not None:
                        fin_angle = i
                    if init_angle is None:
                        init_angle = i
            i += 1

        val_detect_front = True

        if init_angle is not None and fin_angle is not None:
            self.info("Car detected between ia: {}° and fa: {}°".format(init_angle, fin_angle))
            self.scanned_init_angle = init_angle
            self.scanned_fin_angle = fin_angle
        else:
            self.info("No car detected between ia: {}° and fa: {}°".format(237, 267))
            val_detect_front = False

        return val_detect_front         

    def scan_callback(self, data):
        ranges = data.ranges

        twist = Twist()
        
        q1 = ranges[:126] # First quadrant
        q2 = ranges[126:252] # Second quadrant
        q3 = ranges[252:378] # Third quadrant
        q4 = ranges[378:] # Fourth quadrant

        scan_front = q2 + q3
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
    follow_tb3_burger = FollowTB3Burger()
    rclpy.spin(follow_tb3_burger)
    follow_tb3_burger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
