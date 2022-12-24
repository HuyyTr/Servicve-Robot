#!/usr/bin/env python
import rospy
from sr_test.odom_follower import OdomFollower

if __name__ == "__main__":
    rospy.init_node("odom_follower")
    OdomFollower()
    rospy.spin()