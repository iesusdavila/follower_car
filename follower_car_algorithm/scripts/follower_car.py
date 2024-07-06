#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import sys
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class FollowRover(Node):
    def __init__(self):
        super().__init__('follow_rover_node')
        
        argv = sys.argv
        if len(argv) < 2:
            self.get_logger(name_robot).error("It is required to specify the name of the ROVER as an argument to the executable. For example: rover_1")
            sys.exit(1)
        self.name_robot = argv[1]
        self.scan_topic = self.name_robot + "/scan"
        self.cmd_vel_topic = self.name_robot + "/cmd_vel"

        self.sub_scan = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.pub_cmd_vel = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.LIM_DISTANCE = 0.4
        self.LIM_ANGULAR_VELOCITY = 2.0
        self.LIM_LINEAR_VELOCITY = 3.0
        self.GAIN_Kp = 5.0

        self.sup_limit_minl = 30
        self.inf_limit_front = 60
        self.sup_limit_front = 120
        self.sup_limit_maxr = 150

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

    def move_angular(self):
        # Positivo en sentido de las manecillas del reloj
        range_limit_front = self.sup_limit_front - self.inf_limit_front
        range_limit_left = self.inf_limit_front
        range_limit_right = 180 - self.sup_limit_front
        
        angular_vel = 0.0

        if (range_limit_front<25) and (range_limit_left > range_limit_right):
            angular_vel = self.LIM_ANGULAR_VELOCITY
        elif (range_limit_front<25) and (range_limit_left < range_limit_right):
            angular_vel = -self.LIM_ANGULAR_VELOCITY

        return angular_vel

    def scan_front(self, init_angle, fin_angle, ranges):
        # Fourth Quadrant
        is_foq_ia = init_angle < 360 and init_angle > 270 # Init angle
        is_foq_fa = fin_angle < 360 and fin_angle > 270 # Final angle
        # First Quadrant
        is_fiq_ia = init_angle > 0 and init_angle < 90 # Init angle
        is_fiq_fa = fin_angle > 0 and fin_angle < 90 # Final angle

        scan_fwrd_ranges = []

        if is_fiq_ia and is_fiq_fa:
            scan_fwrd_ranges = ranges[init_angle:fin_angle]
        elif is_foq_ia and is_foq_fa:
            scan_fwrd_ranges = ranges[init_angle:fin_angle]
        elif is_foq_ia and is_fiq_fa:
            scan_fwrd_ranges = ranges[:fin_angle] + ranges[init_angle:]
        else:
            scan_fwrd_ranges = ranges[init_angle:fin_angle]
        
        return self.filter_inf(scan_fwrd_ranges)

    def move_linear(self, ranges):
        fwrd_ranges = self.scan_front(self.inf_limit_front, self.sup_limit_front, ranges)

        vel_linear_x = 0.0

        if fwrd_ranges:
            error = min(fwrd_ranges) - self.LIM_DISTANCE
            linear_vel = self.GAIN_Kp * error # Control Proporcional
            
            vel_linear_x = self.limit_velocity(linear_vel, self.LIM_LINEAR_VELOCITY)

        return vel_linear_x
    
    def detect_rover(self, ranges):
        init_angle = None
        fin_angle = None

        # TurtleBot Burger Haciendo pruebas el angulo de deteccion del rover es entre 337 y 23 grados
        i = 337

        scan_ranges = ranges[337:] + ranges[:23]
        if any(self.filter_inf(scan_ranges)):
            self.info("Car in the position of 337° to 23°.")

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
            self.inf_limit_front = init_angle
            self.sup_limit_front = fin_angle

            # mitad_izq = int(init_angle/2)
            # mitad_der = fin_angle + int((180-fin_angle)/2)

            # self.sup_limit_minl = mitad_izq
            # self.sup_limit_maxr = mitad_der
        else:
            self.info("No rover detected.")
            val_detect_front = False

        return val_detect_front         

    def scan_callback(self, data):
        ranges = data.ranges

        twist = Twist()

        scan_front = ranges[:90] + ranges[270:]
        if not (any(self.filter_inf(scan_front))):
            self.info("No car detected. Stopping the car.")
            self.pub_cmd_vel.publish(twist)
            return

        is_detect_rover = self.detect_rover(ranges)
        if is_detect_rover:
            self.info("Car detected.")
            twist.linear.x = self.move_linear(ranges)
            self.info(f'Velocity x: {twist.linear.x}')
            # twist.angular.z = self.move_angular()
            # print("Velocity z: ", twist.angular.z)
            self.pub_cmd_vel.publish(twist)
    
    def info(self, msg):
        self.get_logger().info(msg)
        return

def main(args=None):
    rclpy.init(args=args)
    follow_rover = FollowRover()
    rclpy.spin(follow_rover)
    follow_rover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
