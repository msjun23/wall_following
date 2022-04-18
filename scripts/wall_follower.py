#!/usr/bin/env python3

import rospy
import numpy as np

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class node:
    def __init__(self):
        rospy.init_node('wall_follower', anonymous=True)
        
        self.res = 0.391            # RPLiDAR S1 Resolution in degree
        self.mode = 0
        self.speed = 0.5
        self.turn = -1.0

        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
    def ScanSubscriber(self, data):
        ranges = data.ranges

        # LiDAR range
        #            460(180)
        #               x
        # 690(270)   y--|     230(90)
        #
        #       920(360)|0(0)
        # degree: 150~180
        deg_s_point = int(120 / self.res)
        deg_e_point = int(180 / self.res)
        ranges = ranges[deg_s_point:deg_e_point]    # Front right scan value
        #rospy.loginfo(ranges)
        
        self.WallFollow(ranges)
        
    def WallFollow(self, ranges):
        # ranges: front right ~ front
        # Filtering LiDAR data blocked by robot frame
        ranges = [dist for dist in ranges if dist > 0.5]
        
        cmd_vel = Twist()
        
        conf_dist = 1.0
        # No obstacles: Spiral motion to right side
        if (self.mode == 0):
            speed = self.speed
            turn = self.turn
            rospy.loginfo('Spiral motion ' + str(speed) + ' ' + str(turn))
            if (max(ranges) < conf_dist):
                self.mode = 1
        # Face obstacles: Back step
        elif (self.mode == 1):
            speed = -0.5
            turn = 1.0
            rospy.loginfo('Back step ' + str(speed) + ' ' + str(turn))
            if (min(ranges) > conf_dist):
                self.mode = 0
                self.turn += 0.2
                if (self.turn > 0.0):
                    self.mode = 2
        # Wall following to left side
        elif (self.mode == 2):
            self.turn = -1.0
            speed = self.speed
            turn = 0.0
            rospy.loginfo('Wall following ' + str(speed) + ' ' + str(turn))
            if (max(ranges) < conf_dist):
                self.mode = 1
        
        cmd_vel.linear.x = speed
        cmd_vel.angular.z = turn
        self.pub_vel.publish(cmd_vel)
        
    def QuitHandler(self):
        rospy.loginfo('Shuting down process...')
        
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.0
        cmd_vel.linear.z = 0.0
        
        cmd_vel.angular.x = 0.0
        cmd_vel.angular.y = 0.0
        cmd_vel.angular.z = 0.0
        self.pub_vel.publish(cmd_vel)

    def run(self):
        rospy.Subscriber('/scan', LaserScan, self.ScanSubscriber)
        
        rospy.on_shutdown(self.QuitHandler)
        rospy.spin()

if __name__ == '__main__':
    n = node()
    n.run()
    