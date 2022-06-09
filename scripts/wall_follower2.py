#!/usr/bin/env python3

from atexit import register
import rospy
import numpy as np
import math

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

regions_ = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0
}
state_ = 0
state_dict_ = {
    0: 'fine the wall', 
    1: 'turn left', 
    2: 'follow the wall'
}

class node:
    def __init__(self):
        rospy.init_node('wall_follower', anonymous=True)
        
        self.res = 0.2802            # RPLiDAR S1 Resolution in degree
        self.dist_ref = 0.7         # Reference distance between wall and robot
        self.cnt = 0.0
        self.prev_err = 0.0
        self.ranges = ()
        self.mode = 'stop'
        
        rospy.Subscriber('/umbot_mode', String, self.SetCleaning)
        rospy.Subscriber('/scan', LaserScan, self.ScanSubscriber)
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.cmd_vel = Twist()
        
        rospy.on_shutdown(self.QuitHandler)
        rospy.spin()
        
    def SetCleaning(self, data):
        if data.data == 'cleaning':
            self.mode = 'cleaning'
        else:
            self.mode = 'stop'
            
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.linear.y = 0.0
            self.cmd_vel.linear.z = 0.0
            
            self.cmd_vel.angular.x = 0.0
            self.cmd_vel.angular.y = 0.0
            self.cmd_vel.angular.z = 0.0
            self.pub_vel.publish(self.cmd_vel)
        
    def ScanSubscriber(self, data):
        if self.mode == 'cleaning':
            # LiDAR range
            #            642(180)
            #               x
            # 963(270)   y--|     321(90)
            #
            #       1284(360)|0(0)
            # len(data.ranges) == 1285
            # 127~153 / 501~531 / 769~800 / 1143~1165 data is blocked by robot frame
            
            # for i in range(len(data.ranges)):
            #     if data.ranges[i] < 0.5:
            #         data.ranges[i] = 40.0
            
            self.ranges = data.ranges
            
            global regions_
            regions_ = {
                'right':    min(min(data.ranges[self.Deg2Idx(67.5):self.Deg2Idx(112.5)]), 10),      # 240~401
                
                'fright':   min(min(min(data.ranges[self.Deg2Idx(112.5):self.Deg2Idx(140)]),        # 401~499
                                    min(data.ranges[self.Deg2Idx(150):self.Deg2Idx(157.5)])), 10),  # 535~562
                
                'front':    min(min(data.ranges[self.Deg2Idx(157.5):self.Deg2Idx(202.5)]), 10),     # 562~722
                
                'fleft':    min(min(min(data.ranges[self.Deg2Idx(202.5):self.Deg2Idx(213)]),        # 722~767
                                    min(data.ranges[self.Deg2Idx(225):self.Deg2Idx(247.5)])), 10),  # 802~883
                
                'left':     min(min(data.ranges[self.Deg2Idx(247.5):self.Deg2Idx(292.5)]), 10),     # 883~1043
            }
            
            # print(regions_)
            
            self.TakeAction()
            # print(data.ranges)
            # print(regions_['right'], regions_['fright'], regions_['front'], regions_['fleft'], regions_['left'])
        else:
            pass
        
    def Deg2Idx(self, deg):
        return int(deg / self.res)
    
    def ChangeState(self, state):
        global state_, state_dict_
        
        if state is not state_:
            print('Wall follower - [%s] - %s' % (state, state_dict_[state]))
            state_ = state
            
        if state_ == 0:
            self.FindWall()
        elif state_ == 1:
            self.Rotating()
        elif state_ == 2:
            self.Driving()
        else:
            rospy.logerr('Unknown state!')
        
    def TakeAction(self):
        global regions_
        regions = regions_
        
        state_description = ''
        
        if regions['front'] > self.dist_ref and regions['fleft'] > self.dist_ref and regions['fright'] > self.dist_ref:
            state_description = 'case 1 - nothing: Find wall'
            self.ChangeState(0)     # Find wall
        elif regions['front'] < self.dist_ref and regions['fleft'] > self.dist_ref and regions['fright'] > self.dist_ref:
            state_description = 'case 2 - front: CCW rotation'
            self.ChangeState(1)     # CCW rotation
        elif regions['front'] > self.dist_ref and regions['fleft'] > self.dist_ref and regions['fright'] < self.dist_ref:
            state_description = 'case 3 - fright: Driving'
            self.ChangeState(2)     # Driving
        elif regions['front'] > self.dist_ref and regions['fleft'] < self.dist_ref and regions['fright'] > self.dist_ref:
            state_description = 'case 4 - fleft: Find wall -> CCW rotation'
            self.ChangeState(1)     # Find wall -> CCW rotation: debugging
        elif regions['front'] < self.dist_ref and regions['fleft'] > self.dist_ref and regions['fright'] < self.dist_ref:
            state_description = 'case 5 - front and fright: CCW rotation'
            self.ChangeState(1)     # CCW rotation
        elif regions['front'] < self.dist_ref and regions['fleft'] < self.dist_ref and regions['fright'] > self.dist_ref:
            state_description = 'case 6 - fleft and front: CCW rotation'
            self.ChangeState(1)     # CCW rotation
        elif regions['front'] < self.dist_ref and regions['fleft'] < self.dist_ref and regions['fright'] < self.dist_ref:
            state_description = 'case 7 - fleft and front and fright: CCW rotation'
            self.ChangeState(1)     # CCW rotation
        elif regions['front'] > self.dist_ref and regions['fleft'] < self.dist_ref and regions['fright'] < self.dist_ref:
            state_description = 'case 8 - fleft and fright: Find wall -> CCW rotation'
            self.ChangeState(1)     # Find wall -> CCW rotation: debugging
        else:
            state_description = 'unknown case'
            rospy.loginfo(regions)
        # rospy.loginfo(state_description)
        
    def FindWall(self):
        # self.cmd_vel.linear.x = self.cnt
        # self.cnt += 0.01
        # if self.cnt > 0.22:
        #     self.cnt = 0.22
        self.cmd_vel.linear.x = 0.2
        self.cmd_vel.linear.y = 0.0
        self.cmd_vel.angular.z = -0.2
        
        self.pub_vel.publish(self.cmd_vel)

    def Rotating(self):
        self.cnt = 0.0
        
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.linear.y = 0.0
        self.cmd_vel.angular.z = 0.5
        
        self.pub_vel.publish(self.cmd_vel)

    def Driving(self):
        self.cnt = 0.0
        
        # Follow the wall
        # b = self.ranges[self.Deg2Idx(90)]
        # a = self.ranges[self.Deg2Idx(150)]
        a = min(self.ranges[self.Deg2Idx(85):self.Deg2Idx(95)])
        b = min(self.ranges[self.Deg2Idx(145):self.Deg2Idx(155)])
        theta = 60.0 / 180.0 * np.pi
        alpha = np.arctan2((a*np.cos(theta) - b), (a*np.sin(theta)))
        #print(alpha / np.pi * 180.0)
        
        Dt = b*np.cos(alpha)
        #print(Dt)
        
        speed = 0.22
        L = speed / 10        # Distance that robot drives to straightly when linear speed is constant = speed / 10
        Dt1 = Dt + L*np.sin(alpha)
        #print(Dt1)
        
        kp = 0.1
        kd = 0.01
        dist_err = self.dist_ref - Dt1
        turn = kp * dist_err + kd * (dist_err - self.prev_err)
        if (turn > 0.5):
            turn = 0.5
        elif (turn < -0.5):
            turn = -0.5
        self.prev_err = dist_err
        
        self.cmd_vel.linear.x = speed
        self.cmd_vel.linear.y = turn * 3
        self.cmd_vel.angular.z = turn
        
        # rospy.loginfo('Following the wall. ' + 'error: ' + str(dist_err) + ' | Dt1: ' + str(Dt1) + ' | Dt: ' + str(Dt) + ' | alpha: ' + str(alpha) + ' | theta: ' + str(theta) + ' | a: ' + str(a) + ' | b: ' + str(b))
        rospy.loginfo('Following the wall. ' + 'error: ' + str(dist_err) + '\n speed: ' + str(speed) + ' turn: ' + str(turn))
        self.pub_vel.publish(self.cmd_vel)
        
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
    