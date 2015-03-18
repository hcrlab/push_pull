#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from trajopt_test.srv import *
import tf

if __name__ == "__main__":
    rospy.init_node('trajopt_server_tester')
    rospy.wait_for_service('trajopt_navigate')
    s = rospy.ServiceProxy('trajopt_navigate', TrajoptNavigate)

    pose_list = []

    goal = PoseStamped()
    goal.pose.position.x = 0.5
    goal.pose.position.y = -0.2
    goal.pose.position.z = 0.5
    goal.pose.orientation.w = 1
    pose_list.append(goal)

    goal1 = PoseStamped()
    
    roll = 0.0
    pitch = 0.0
    yaw = 1.57
    quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    
    goal1.pose.position.x = 0.3
    goal1.pose.position.y = 0.2
    goal1.pose.position.z = 0.7
    goal1.pose.orientation.x = quat[0]
    goal1.pose.orientation.y = quat[1]
    goal1.pose.orientation.z = quat[2]
    goal1.pose.orientation.w = quat[3]

    pose_list.append(goal1)

    goal2 = PoseStamped()

    roll = 0.0
    pitch = 0.0
    yaw = 1.57
    quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

    goal2.pose.position.x = 0.3
    goal2.pose.position.y = 0.1
    goal2.pose.position.z = 0.7
    goal2.pose.orientation.x = quat[0]
    goal2.pose.orientation.y = quat[1]
    goal2.pose.orientation.z = quat[2]
    goal2.pose.orientation.w = quat[3]

    pose_list.append(goal2)

    count = 0
    for pose in pose_list:
        try:
            if count == 0:
                b = s(pose, True)
            else:
                b = s(pose, False)
            print "Yay: " + str(b)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
        count = count + 1
