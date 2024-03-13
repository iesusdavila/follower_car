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

        self.LIM_DISTANCE = 0.75
        self.LIM_ANGULAR_VELOCITY = 1.0
        self.LIM_LINEAR_VELOCITY = 1.0
        self.GAIN_Kp = 2.0

    def limit_velocity(self, vel, lim_vel):
        if vel > lim_vel:
            vel = lim_vel
        elif vel < -lim_vel:
            vel = -lim_vel
        return vel
    
    def filter_inf(self, ranges, start, end):
        return [val for val in ranges[start:end] if not math.isinf(val)]    

    def scan_callback(self, data):
        ranges = data.ranges

        minl_ranges = self.filter_inf(ranges, 0, 30)
        maxl_ranges = self.filter_inf(ranges, 30, 60)
        fwrd_ranges = self.filter_inf(ranges, 60, 120)
        maxr_ranges = self.filter_inf(ranges, 120, 150)
        minr_ranges = self.filter_inf(ranges, 150, 180)

        twist = Twist()

        if ranges:
            error = min(ranges) - self.LIM_DISTANCE
            twist.linear.x = self.GAIN_Kp * error
            
        if minl_ranges and (min(minl_ranges) > self.LIM_DISTANCE):
            twist.angular.z = -self.LIM_ANGULAR_VELOCITY*2
            rospy.loginfo("minl_ranges: {}".format(min(minl_ranges)))

        if maxl_ranges and (min(maxl_ranges) > self.LIM_DISTANCE):
            twist.angular.z = -self.LIM_ANGULAR_VELOCITY
            rospy.loginfo("maxl_ranges: {}".format(min(maxl_ranges)))

        if fwrd_ranges and (min(fwrd_ranges) > self.LIM_DISTANCE):
            rospy.loginfo("fwrd_ranges: {}".format(min(fwrd_ranges)))

        if maxr_ranges and (min(maxr_ranges) > self.LIM_DISTANCE):
            twist.angular.z = self.LIM_ANGULAR_VELOCITY
            rospy.loginfo("maxr_ranges: {}".format(min(maxr_ranges)))

        if minr_ranges and (min(minr_ranges) > self.LIM_DISTANCE):
            twist.angular.z = self.LIM_ANGULAR_VELOCITY*2
            rospy.loginfo("minr_ranges: {}".format(min(minr_ranges)))

        self.pub_cmd_vel.publish(twist)

if __name__ == '__main__':
    follow_rover = FollowRover()
    rospy.spin()
