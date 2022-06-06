#!/usr/bin/env python3

import rospy
import numpy as np
import math

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class node:
    def __init__(self):
        rospy.init_node('scan_test', anonymous=True)
        
        rospy.Subscriber('/scan', LaserScan, self.ScanSubscriber)
        # self.pub_scan = rospy.Publisher('/scan_frame', LaserScan, queue_size=10)
        
        rospy.spin()
        
    def ScanSubscriber(self, data):
        # LiDAR range
        #            642(180)
        #               x
        # 963(270)   y--|     321(90)
        #
        #       1284(360)|0(0)
        # 127~153 / 501~531 / 771~795 / 1143~1165 data is blocked by robot frame
        # 1285
        
        # for i in range(len(data.ranges)):
        #     if data.ranges[i] < 0.5:
        #         data.ranges[i] = 40.0
        ranges = data.ranges
        
        l = []
        for s in range(len(ranges)):
            if ranges[s] < 0.5:
                l.append(s)
        print(l)
        
if __name__ == '__main__':
    n = node()
    