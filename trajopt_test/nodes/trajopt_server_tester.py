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
        goal.pose.position.x = 0.6
        goal.pose.position.x = 0.024
        goal.pose.position.x = 1.43
        s(goal, True)
        print "Yay!"
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
