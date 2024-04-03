#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class FollowRover:
    def __init__(self):
        rospy.init_node('follow_rover_node', anonymous=True)
        
        argv = rospy.myargv(argv=sys.argv)
        if len(argv) < 2:
            rospy.logerr("It is required to specify the name of the ROVER as an argument to the executable. For example: rover_1")
            sys.exit(1)
        self.name_robot = argv[1]
        self.scan_topic = self.name_robot + "/scan"
        self.cmd_vel_topic = self.name_robot + "/cmd_vel"

        self.sub_scan = rospy.Subscriber(self.scan_topic, LaserScan, self.scan_callback)
        self.pub_cmd_vel = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)

        self.LIM_DISTANCE = 0.50
        self.LIM_ANGULAR_VELOCITY = 1.0
        self.LIM_LINEAR_VELOCITY = 1.0
        self.GAIN_Kp = 2.0

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

    def move_angular(self, ranges):
        angular_vel = 0.0
        range_limit_front = self.sup_limit_front - self.inf_limit_front
        range_limit_left = self.inf_limit_front
        range_limit_right = 180 - self.sup_limit_front
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
            linear_vel = self.GAIN_Kp * error
            
            vel_linear_x = self.limit_velocity(linear_vel, self.LIM_LINEAR_VELOCITY)

        return vel_linear_x
    
    def detect_rover(self, ranges):
        init_angle = None
        fin_angle = None

        i = 67
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
            rospy.loginfo("Rover detected between {} and {} degrees".format(init_angle, fin_angle))
            self.inf_limit_front = init_angle
            self.sup_limit_front = fin_angle

            mitad_izq = int(init_angle/2)
            mitad_der = fin_angle + int((180-fin_angle)/2)

            self.sup_limit_minl = mitad_izq
            self.sup_limit_maxr = mitad_der
        else:
            rospy.loginfo("No rover detected.")
            val_detect_front = False

        return val_detect_front         

    def scan_callback(self, data):
        ranges = data.ranges

        twist = Twist()

        if not any(self.filter_inf(ranges, 0, 180)):
            rospy.loginfo("No objects detected. Stopping the robot.")
            self.pub_cmd_vel.publish(twist)
            return
        
        is_detect_rover = self.detect_rover(ranges)
        if is_detect_rover:
            print("Velocity x: ", self.move_linear(ranges))
            twist.linear.x = self.move_linear(ranges)
            twist.angular.z = self.move_angular(ranges)
            self.pub_cmd_vel.publish(twist)

if __name__ == '__main__':
    follow_rover = FollowRover()
    rospy.spin()
