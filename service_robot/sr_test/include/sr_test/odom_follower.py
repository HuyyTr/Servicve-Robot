#!/usr/bin/env python

import rospy
import math
from tf.transformations import euler_from_quaternion

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion, Point
from sr_test.srv import FollowerInfo, FollowerInfoResponse

class OdomFollower:
    def __init__(self):
        self.max_linear_velocity = 0.2
        self.min_linear_velocity = 0.05
        self.max_angular_velocity = 0.2
        self.min_angular_velocity = 0.03
        self.isSetup = False
        self.verticals = []
        self.quat = Quaternion()
        self.kp = 0.6
        self.delta_x_translation = 0.0
        self.delta_y_translation = 0.0
        self.delta_rotation = 0.0
        self.error = 0.1
        self.angle = 0.0
        self.x = 0.0
        self.y = 0.0
        self.x_prev = 0.0
        self.y_prev = 0.0
        self.angle_prev = 0.0
        self.loop_count = 0
        self.clockwise = True
        self.size = 0.0
        self.shape = 0 # 0: SQUARE, 1: CIRCLE
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odomCallBack, queue_size=10)
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.follower_server = rospy.Service("/odom_follower", FollowerInfo, self.odomFollowerCallBack)
    
    def odomCallBack(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.quat = msg.pose.pose.orientation    
        q = [self.quat.x, self.quat.y, self.quat.z, self.quat.w]
        (roll, pitch, yaw) = euler_from_quaternion(q)
        self.angle = yaw
        
        if(not self.isSetup):
            first_point = Point()
            first_point.x = self.x
            first_point.y = self.y
            self.x_prev = self.x
            self.y_prev = self.y
            self.angle_prev = self.angle
            self.verticals.append(first_point)
            self.isSetup = True
        
        self.delta_x_translation += (abs(self.x) - abs(self.x_prev))
        self.delta_y_translation += (abs(self.y) - abs(self.y_prev))
        self.delta_rotation += (abs(self.angle) - abs(self.angle_prev))
        self.x_prev = self.x
        self.y_prev = self.y
        self.angle_prev = self.angle
        
        rospy.loginfo("delta_x_translation: {} - delta_y_translation: {} - delta_rotation: {}".format(self.delta_x_translation, self.delta_y_translation, self.delta_rotation))
        # rospy.loginfo(f"X: {self.x} - Y: {self.y} - Yaw: {self.angle} .")
    
    def odomFollowerCallBack(self, req):
        self.isSetup = False
        self.shape = req.shape
        self.clockwise = req.clockwise
        self.size = req.size
        self.loop_count = req.loop_number
        r = rospy.Rate(10)
        if(self.shape == 0):
            if(self.clockwise):
                
                twist_msg = Twist()
                while (True):
                    if (self.loop_count -1 >= 0):
                        rospy.loginfo("Translation 1")
                        #Translation +y
                        while(True):
                            if(abs(self.size - self.delta_x_translation) > self.error):
                                twist_msg.linear.x = self.kp * (self.size - self.delta_x_translation)
                                if(twist_msg.linear.x > self.max_linear_velocity):
                                    twist_msg.linear.x = self.max_linear_velocity
                                if(twist_msg.linear.x < self.min_linear_velocity):
                                    twist_msg.linear.x = self.min_linear_velocity
                                self.vel_pub.publish(twist_msg)
                                r.sleep()
                            else:                        
                                twist_msg.linear.x = 0.0
                                self.vel_pub.publish(twist_msg)
                                self.delta_x_translation = 0.0
                                
                                second_point = Point()
                                second_point.x = self.x
                                second_point.y = self.y
                                self.verticals.append(second_point)
                                break
                        
                        rospy.loginfo("Rotation 1")
                        #Rotation +pi/2
                        while(True):
                            if(abs(math.pi/2 - self.delta_rotation)> self.error/3):
                                twist_msg.angular.z = self.kp * (math.pi/2 - self.delta_rotation)
                                if(twist_msg.angular.z > self.max_angular_velocity):
                                    twist_msg.angular.z = self.max_angular_velocity
                                if(twist_msg.angular.z < self.min_angular_velocity):
                                    twist_msg.angular.z = self.min_angular_velocity
                                self.vel_pub.publish(twist_msg)
                                r.sleep()
                            else:
                                twist_msg.angular.z = 0.0
                                self.vel_pub.publish(twist_msg)
                                self.delta_rotation = 0.0
                                break
                        
                        rospy.loginfo("Translation 2")
                        #Translation +x
                        while(True):
                            if(abs(self.size - self.delta_y_translation) > self.error):
                                twist_msg.linear.x = self.kp * (self.size - self.delta_y_translation)
                                if(twist_msg.linear.x > self.max_linear_velocity):
                                    twist_msg.linear.x = self.max_linear_velocity
                                if(twist_msg.linear.x < self.min_linear_velocity):
                                    twist_msg.linear.x = self.min_linear_velocity
                                self.vel_pub.publish(twist_msg)
                                r.sleep()
                            else:
                                twist_msg.linear.x = 0.0
                                self.vel_pub.publish(twist_msg)
                                self.delta_y_translation = 0.0
                                
                                third_point = Point()
                                third_point.x = self.x
                                third_point.y = self.y
                                self.verticals.append(third_point)
                                break

                        rospy.loginfo("Rotation 2")  
                        #Rotation +pi/2
                        while(True):
                            if(abs(math.pi/2 - self.delta_rotation)> self.error/3):
                                twist_msg.angular.z = self.kp * (math.pi/2 - self.delta_rotation)
                                if(twist_msg.angular.z > self.max_angular_velocity):
                                    twist_msg.angular.z = self.max_angular_velocity
                                if(twist_msg.angular.z < self.min_angular_velocity):
                                    twist_msg.angular.z = self.min_angular_velocity
                                self.vel_pub.publish(twist_msg)
                                r.sleep()
                            else:
                                twist_msg.angular.z = 0.0
                                self.vel_pub.publish(twist_msg)
                                self.delta_rotation = 0.0
                                break
                            
                        rospy.loginfo("Translation 3")
                        #Translation -y
                        while(True):
                            if(abs(self.size - abs(self.delta_x_translation)) > self.error):
                                twist_msg.linear.x = self.kp * (self.size - abs(self.delta_x_translation))
                                if(twist_msg.linear.x > self.max_linear_velocity):
                                    twist_msg.linear.x = self.max_linear_velocity
                                if(twist_msg.linear.x < self.min_linear_velocity):
                                    twist_msg.linear.x = self.min_linear_velocity
                                self.vel_pub.publish(twist_msg)
                                r.sleep()
                            else:
                                twist_msg.linear.x = 0.0
                                self.vel_pub.publish(twist_msg)
                                self.delta_x_translation = 0.0
                                
                                fourth_point = Point()
                                fourth_point.x = self.x
                                fourth_point.y = self.y
                                self.verticals.append(fourth_point)
                                # rospy.loginfo(f"delta_rotation: {self.delta_rotation}")
                                break

                        rospy.loginfo("Rotation 3")                
                        #Rotation +pi/2
                        while(True):
                            # rospy.loginfo(f"delta_rotation: {self.delta_rotation}")
                            if(abs(math.pi/2 - abs(self.delta_rotation))> self.error/3):
                                twist_msg.angular.z = self.kp * (math.pi/2 - abs(self.delta_rotation))
                                if(twist_msg.angular.z > self.max_angular_velocity):
                                    twist_msg.angular.z = self.max_angular_velocity
                                if(twist_msg.angular.z < self.min_angular_velocity):
                                    twist_msg.angular.z = self.min_angular_velocity
                                self.vel_pub.publish(twist_msg)
                                r.sleep()
                            else:
                                twist_msg.angular.z = 0.0
                                self.vel_pub.publish(twist_msg)
                                self.delta_rotation = 0.0
                                break
                        
                        rospy.loginfo("Translation 4")
                        #Translation -x
                        while(True):
                            if(abs(self.size - abs(self.delta_y_translation)) > self.error):
                                twist_msg.linear.x = self.kp * (self.size - abs(self.delta_y_translation))
                                if(twist_msg.linear.x > self.max_linear_velocity):
                                    twist_msg.linear.x = self.max_linear_velocity
                                if(twist_msg.linear.x < self.min_linear_velocity):
                                    twist_msg.linear.x = self.min_linear_velocity
                                self.vel_pub.publish(twist_msg)
                                r.sleep()
                            else:
                                twist_msg.linear.x = 0
                                self.vel_pub.publish(twist_msg)
                                self.delta_y_translation = 0.0
                                
                                end_point = Point()
                                end_point.x = self.x
                                end_point.y = self.y
                                self.verticals.append(end_point)
                                break
                        
                        rospy.loginfo("Rotation 4")                
                        #Rotation +pi/2
                        while(True):
                            # rospy.loginfo(f"delta_rotation: {self.delta_rotation}")
                            if(abs(math.pi/2 - abs(self.delta_rotation))> self.error/3):
                                twist_msg.angular.z = self.kp * (math.pi/2 - abs(self.delta_rotation))
                                if(twist_msg.angular.z > self.max_angular_velocity):
                                    twist_msg.angular.z = self.max_angular_velocity
                                if(twist_msg.angular.z < self.min_angular_velocity):
                                    twist_msg.angular.z = self.min_angular_velocity
                                self.vel_pub.publish(twist_msg)
                                r.sleep()
                            else:
                                twist_msg.angular.z = 0.0
                                self.vel_pub.publish(twist_msg)
                                self.delta_rotation = 0.0
                                break
                        self.loop_count -= 1
                    else:
                        break
                    
        res = FollowerInfoResponse()
        res.verticals = self.verticals
        return res