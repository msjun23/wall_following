#!/usr/bin/env python3

import rospy
import numpy as np

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class node:
    def __init__(self):
        rospy.init_node('wall_follower', anonymous=True)
        
        self.res = 0.391            # RPLiDAR S1 Resolution in degree
        self.mode = 0
        self.cnt = 0
        self.dist_ref = 0.5         # Reference distance between wall and robot
        self.prev_err = 0.0
        
        rospy.Subscriber('/umbot_mode', String, self.SetCleaning)
        rospy.Subscriber('/scan', LaserScan, self.ScanSubscriber)
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.cmd_vel = Twist()
        
        rospy.on_shutdown(self.QuitHandler)
        rospy.spin()
        
    def SetCleaning(self, data):
        rospy.loginfo(data)
        if (self.mode < 0 and data.data == 'cleaning'):
            self.mode = 0
        else:
            self.QuitHandler()
            self.mode = -1
        
    def Deg2Idx(self, deg):
        return int(deg / self.res)
    
    def ToTheWall(self, nearest_dir, dist_err):
        kp = 1
        kd = 0.001
        
        speed = kp * dist_err + kd * (dist_err - self.prev_err)
        if (speed > 0.22):
            speed = 0.22
        elif (speed < -0.22):
            speed = -0.22
        self.prev_err = dist_err
        
        if (nearest_dir == 0):                  # Approach to right wall
            # self.cmd_vel.linear.x = 0.0
            self.cmd_vel.linear.y = speed
            rospy.loginfo('Approaching to right wall. ' + 'error: ' + str(dist_err))
        elif (nearest_dir == 1):                # Approach to front wall
            self.cmd_vel.linear.x = -speed
            # self.cmd_vel.linear.y = 0.0
            rospy.loginfo('Approaching to front wall. ' + 'error: ' + str(dist_err))
        elif (nearest_dir == 2):                # Approach to left wall
            # self.cmd_vel.linear.x = 0.0
            self.cmd_vel.linear.y = -speed
            rospy.loginfo('Approaching to left wall. ' + 'error: ' + str(dist_err))
        elif (nearest_dir == 3):                # Approach to back wall
            self.cmd_vel.linear.x = speed
            # self.cmd_vel.linear.y = 0.0
            rospy.loginfo('Approaching to back wall. ' + 'error: ' + str(dist_err))
            
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
        self.cmd_vel.angular.z = 0.5
        
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
        
        L = 0.022        # Distance that robot drive to straightly when linear speed is 0.22[m/s]
        Dt1 = Dt + L*np.sin(alpha)
        #print(Dt1)
        
        kp = 0.5
        kd = 0.1
        speed = 0.22
        dist_err = self.dist_ref - Dt1
        turn = kp * dist_err + kd * (dist_err - self.prev_err)
        if (turn > 0.1):
            turn = 0.1
        elif (turn < -0.1):
            turn = -0.1
        self.prev_err = dist_err
        
        self.cmd_vel.linear.x = speed
        self.cmd_vel.linear.y = turn
        self.cmd_vel.angular.z = turn
        
        rospy.loginfo('Following the wall. ' + 'error: ' + str(dist_err))
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
            dist_r = sum(ranges[self.Deg2Idx(60):self.Deg2Idx(120)]) \
                    / len(ranges[self.Deg2Idx(60):self.Deg2Idx(120)])       # Avarage of right side scan data
            dist_f = sum(ranges[self.Deg2Idx(150):self.Deg2Idx(210)]) \
                    / len(ranges[self.Deg2Idx(150):self.Deg2Idx(210)])      # Avarage of front side scan data
            dist_l = sum(ranges[self.Deg2Idx(240):self.Deg2Idx(300)]) \
                    / len(ranges[self.Deg2Idx(240):self.Deg2Idx(300)])      # Avarage of left side scan data
            dist_b = (sum(ranges[self.Deg2Idx(0):self.Deg2Idx(30)]) + sum(ranges[self.Deg2Idx(330):self.Deg2Idx(360)])) \
                    / (len(ranges[self.Deg2Idx(0):self.Deg2Idx(30)]) + len(ranges[self.Deg2Idx(330):self.Deg2Idx(360)]))    # Avarage of back side scan data
            dist = [dist_r, dist_f, dist_l, dist_b]
            nearest_dir = dist.index(min(dist))     # 0: right, 1: front, 2: left, 3: back
            
            dist_err = self.dist_ref - dist[nearest_dir]
            offset = 1
            
            if (abs(dist_err) > 0.1):
                print(np.mean(ranges[self.Deg2Idx(90):self.Deg2Idx(120)]), np.mean(ranges[self.Deg2Idx(60):self.Deg2Idx(90)]) + offset)
                if (nearest_dir == 0 and np.mean(ranges[self.Deg2Idx(90):self.Deg2Idx(120)]) > np.mean(ranges[self.Deg2Idx(60):self.Deg2Idx(90)]) + offset):
                    rospy.loginfo('sibal corner')
                    pass
                else:
                    # Approaching to the wall is first
                    self.ToTheWall(nearest_dir, dist_err)
            else:
                if (nearest_dir == 0):
                    self.Driving(ranges)
                else:
                    self.mode = 1
        elif (self.mode == 1):
            if (self.Assigned(ranges[self.Deg2Idx(60):self.Deg2Idx(120)]) and self.cnt >= 10):
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

if __name__ == '__main__':
    n = node()
    