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
            self.get_logger().error("It is required to specify the name of the ROVER as an argument to the executable. For example: rover_1")
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
    
    def filter_inf(self, ranges, start, end):
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

    def move_linear(self, ranges):
        fwrd_ranges = self.filter_inf(ranges, self.inf_limit_front, self.sup_limit_front)

        vel_linear_x = 0.0

        if fwrd_ranges:
            error = min(fwrd_ranges) - self.LIM_DISTANCE
            linear_vel = self.GAIN_Kp * error # Control P (Interesante)
            
            vel_linear_x = self.limit_velocity(linear_vel, self.LIM_LINEAR_VELOCITY)

        return vel_linear_x
    
    def detect_rover(self, ranges):
        init_angle = None
        fin_angle = None

        i = 67 # Haciendo pruebas el angulo de deteccion del rover es entre 67 y 113 grados
        for scan in ranges[67:113]:
            if not math.isinf(scan):
                if scan < self.LIM_DISTANCE*2:
                    if init_angle is not None:
                        fin_angle = i
                    if init_angle is None:
                        init_angle = i
            i += 1

        val_detect_front = True

        if init_angle is not None and fin_angle is not None:
            print("Rover detected between {} and {} degrees".format(init_angle, fin_angle))
            self.inf_limit_front = init_angle
            self.sup_limit_front = fin_angle

            mitad_izq = int(init_angle/2)
            mitad_der = fin_angle + int((180-fin_angle)/2)

            self.sup_limit_minl = mitad_izq
            self.sup_limit_maxr = mitad_der
        else:
            print("No rover detected.")
            val_detect_front = False

        return val_detect_front         

    def scan_callback(self, data):
        ranges = data.ranges

        twist = Twist()

        if not any(self.filter_inf(ranges, 0, 180)):
            print("No objects detected. Stopping the robot.")
            self.pub_cmd_vel.publish(twist)
            return
        
        is_detect_rover = self.detect_rover(ranges)
        if is_detect_rover:
            print("Velocity x: ", self.move_linear(ranges))
            twist.linear.x = self.move_linear(ranges)
            twist.angular.z = self.move_angular()
            self.pub_cmd_vel.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    follow_rover = FollowRover()
    rclpy.spin(follow_rover)
    follow_rover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
