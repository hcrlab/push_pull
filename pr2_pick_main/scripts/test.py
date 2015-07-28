#!usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from pr2_pick_manipulation.srv import MoveArmIk, MoveArmIkRequest

client = rospy.ServiceProxy('move_arm_ik', MoveArmIk)
goal = PoseStamped()
goal.header.frame_id = "l_wrist_roll_link"
goal.pose.position.x = 0.1
goal.pose.orientation.w = 1.0
resp = client(goal=goal, arm = MoveArmIkRequest.LEFT_ARM)
