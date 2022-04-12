#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class node:
    def __init__(self):
        rospy.init_node('wall_follower', anonymous=True)

        self.pub_scan = rospy.Publisher('/scan_FR', LaserScan, queue_size=1)
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
    def ScanSubscriber(self, data):
        ranges = data.ranges

        # LiDAR range
        #         460
        #          x
        # 690   y--|     230
        #
        #       920|0
        # degree: 330~360
        ranges = ranges[383:460]    # Front right scan value
        #rospy.loginfo(ranges)
        
        # pub_data = data
        # pub_data.ranges = ranges
        # self.pub_scan.publish(pub_data)
        
        self.WallFollow(ranges)
        
    def WallFollow(self, ranges):
        cmd_vel = Twist()
        
        cmd_vel.linear.x = 0.5
        cmd_vel.linear.y = 0.0
        cmd_vel.linear.z = 0.0
        
        cmd_vel.angular.x = 0.0
        cmd_vel.angular.y = 0.0
        cmd_vel.angular.z = 0.0
        
        self.pub_vel.publish(cmd_vel)

    def run(self):
        rospy.Subscriber('/scan', LaserScan, self.ScanSubscriber)
        
        rospy.spin()

if __name__ == '__main__':
    n = node()
    n.run()
    