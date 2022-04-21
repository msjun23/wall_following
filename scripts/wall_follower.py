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
        self.cnt = 0
        self.kp = 1
        self.kd = 0.001
        self.dist_ref = 0.5         # Reference distance between wall and robot
        self.prev_err = 0.0
        
        self.cmd_vel = Twist()
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
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
        
    def Assigned(self, ranges):
        res = 60.0 / len(ranges)
        
        x_coord = []
        for i in range(len(ranges)):
            x_coord.append(ranges[i] * np.cos((-30.0 + res * i) / 180 * np.pi))
            
        x_std = np.std(x_coord)
        if (x_std < 0.01):
            return True
        return False
        
    def Rotating(self):
        # Rotate until attach right side of robot to the wall
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.linear.y = 0.0
        self.cmd_vel.angular.z = 1.0
            
        rospy.loginfo('Rotating... ' + str(self.cnt))
        self.pub_vel.publish(self.cmd_vel)
        
    def Driving(self, ranges):
        # Follow the wall
        
        b = ranges[self.Deg2Idx(90)]
        a = ranges[self.Deg2Idx(150)]
        theta = 60.0 / 180.0 * np.pi
        alpha = np.arctan((a*np.cos(theta) - b) / (a*np.sin(theta)))
        #print(alpha / np.pi * 180.0)
        
        Dt = b*np.cos(alpha)
        #print(Dt)
        
        L = 0.05        # Distance that robot drive to straightly when linear speed is 0.5
        Dt1 = Dt + L*np.sin(alpha)
        #print(Dt1)
        
        speed = 0.5
        dist_err = self.dist_ref - Dt1
        turn = self.kp * dist_err + self.kd * (dist_err - self.prev_err)
        self.prev_err = dist_err
        
        self.cmd_vel.linear.x = speed
        self.cmd_vel.linear.y = 0.0
        self.cmd_vel.angular.z = turn
        
        rospy.loginfo('Following the wall')
        self.pub_vel.publish(self.cmd_vel)
        
    def ScanSubscriber(self, data):
        # LiDAR range
        #            460(180)
        #               x
        # 690(270)   y--|     230(90)
        #
        #       920(360)|0(0)
        # 34~41 / 139~146 / 214~221 / 318~326 degree is blocked by robot frame
        # for i in range(360):
        #     print(i, ranges[self.Deg2Idx(i)])
        ranges = data.ranges
        
        if (self.mode == 0):
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
            
            dist_err = self.dist_ref - dist[nearest_dir]
            
            if (abs(dist_err) > 0.1):
                # Approaching to the wall is first
                self.ToTheWall(nearest_dir, dist_err)
            else:
                if (nearest_dir == 0):
                    self.Driving(ranges)
                else:
                    self.mode = 1
        elif (self.mode == 1):
            if (self.Assigned(ranges[self.Deg2Idx(60):self.Deg2Idx(120)]) and self.cnt >= 50):
                # Right side of robot is attached to the wall
                self.mode = 0
                self.cnt = 0
            else:
                # Other side of robot is attached to the wall
                self.cnt += 1
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
    