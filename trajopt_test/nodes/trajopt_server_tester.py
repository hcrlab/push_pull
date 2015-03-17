#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from trajopt_test.srv import *

if __name__ == "__main__":
    rospy.init_node('trajopt_server_tester')
    rospy.wait_for_service('trajopt_navigate')
    try:
        s = rospy.ServiceProxy('trajopt_navigate', TrajoptNavigate)
        goal = PoseStamped()
        goal.pose.position.x = 0.5
        goal.pose.position.y = -0.2
        goal.pose.position.z = 0.5
        goal.pose.orientation.w = 1
        s(goal, True)
        print "Yay!"
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
