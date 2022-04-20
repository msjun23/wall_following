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
        self.kp = 1
        self.kd = 0.001
        self.prev_err = 0.0
        self.speed = 0.1
        self.turn = -1.0

        self.cmd_vel = Twist()
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
        # deg_s_point = int(120 / self.res)
        # deg_e_point = int(240 / self.res)
        # deg_s_point = int(60 / self.res)
        # deg_e_point = int(120 / self.res)
        # ranges = ranges[deg_s_point:deg_e_point]    # Front right scan value
        #rospy.loginfo(ranges)
        
        self.WallFollow(ranges)
        
    def Deg2Idx(self, deg):
        return int(deg / self.res)
    
    def ToTheWall(self, nearest_dir, dist_err):
        #self.cmd_vel.angular.z = 0.0
        
        if (nearest_dir == 0):                  # Approach to right wall
            speed = self.kp * dist_err + self.kd * (dist_err - self.prev_err)
            self.prev_err = dist_err
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.linear.y = speed
            rospy.loginfo('Approaching to right wall')
        elif (nearest_dir == 1):                # Approach to front wall
            speed = self.kp * dist_err + self.kd * (dist_err - self.prev_err)
            self.prev_err = dist_err
            self.cmd_vel.linear.x = -speed
            self.cmd_vel.linear.y = 0.0
            rospy.loginfo('Approaching to front wall')
        elif (nearest_dir == 2):                # Approach to left wall
            speed = self.kp * dist_err + self.kd * (dist_err - self.prev_err)
            self.prev_err = dist_err
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.linear.y = -speed
            rospy.loginfo('Approaching to left wall')
        elif (nearest_dir == 3):                # Approach to back wall
            speed = self.kp * dist_err + self.kd * (dist_err - self.prev_err)
            self.prev_err = dist_err
            self.cmd_vel.linear.x = speed
            self.cmd_vel.linear.y = 0.0
            rospy.loginfo('Approaching to back wall')
            
        self.pub_vel.publish(self.cmd_vel)
        
    def Rotating(self):
        # Rotate until attach right side of robot to the wall
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.linear.y = 0.0
        self.cmd_vel.angular.z = 0.5
            
        rospy.loginfo('Rotating...')
        self.pub_vel.publish(self.cmd_vel)
        
    def Driving(self):
        # Follow the wall
        self.cmd_vel.linear.x = 0.5
        self.cmd_vel.angular.z = 0.0
        
        rospy.loginfo('Following the wall')
        self.pub_vel.publish(self.cmd_vel)
    
    def WallFollow(self, ranges):
        if (self.mode == 0):
            # ranges: front right ~ front
            # Filtering LiDAR data blocked by robot frame
            #ranges = [dist for dist in ranges if dist > 0.3]
                    
            # Find nearest wall
            dist_r = dist_r = sum(ranges[self.Deg2Idx(60):self.Deg2Idx(120)]) \
                            / len(ranges[self.Deg2Idx(60):self.Deg2Idx(120)])       # Avarage of right side scan data
            dist_f = dist_f = sum(ranges[self.Deg2Idx(150):self.Deg2Idx(210)]) \
                            / len(ranges[self.Deg2Idx(150):self.Deg2Idx(210)])      # Avarage of front side scan data
            dist_l = dist_l = sum(ranges[self.Deg2Idx(240):self.Deg2Idx(300)]) \
                            / len(ranges[self.Deg2Idx(240):self.Deg2Idx(300)])      # Avarage of left side scan data
            dist_b = dist_b = (sum(ranges[self.Deg2Idx(0):self.Deg2Idx(30)]) + sum(ranges[self.Deg2Idx(330):self.Deg2Idx(360)])) \
                            / (len(ranges[self.Deg2Idx(0):self.Deg2Idx(30)]) + len(ranges[self.Deg2Idx(330):self.Deg2Idx(360)]))    # Avarage of back side scan data
            dist = [dist_r, dist_f, dist_l, dist_b]
            nearest_dir = dist.index(min(dist))     # 0: right, 1: front, 2: left, 3: back
            
            dist_ref = 1.0                          # Reference distance between wall and robot
            dist_err = dist_ref - dist[nearest_dir]
            
            if (abs(dist_err) > 0.1):
                # Approaching to the wall is first
                self.ToTheWall(nearest_dir, dist_err)
            else:
                # if (nearest_dir == 0):              # Right side of robot is attached to the wall
                #     self.Driving()
                # else:                               # Other side of robot is attached to the wall
                if (abs(ranges[self.Deg2Idx(90)] - 1.0) < 0.1):
                    # Right side of robot is attached to the wall
                    self.Driving()
                else:
                    self.mode = 1
        else:
            if (abs(ranges[self.Deg2Idx(90)] - 1.0) < 0.1):
                # Right side of robot is attached to the wall
                self.mode = 0
            else:
                self.Rotating()
        
    def QuitHandler(self):
        rospy.loginfo('Shuting down process...')
        
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.linear.y = 0.0
        self.cmd_vel.linear.z = 0.0
        
        self.cmd_vel.angular.x = 0.0
        self.cmd_vel.angular.y = 0.0
        self.cmd_vel.angular.z = 0.0
        self.pub_vel.publish(self.cmd_vel)

    def run(self):
        rospy.Subscriber('/scan', LaserScan, self.ScanSubscriber)
        
        rospy.on_shutdown(self.QuitHandler)
        rospy.spin()

if __name__ == '__main__':
    n = node()
    n.run()
    