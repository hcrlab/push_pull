#!/usr/bin/env python

import rospy
import sys
from pr2_pick_manipulation.srv import *
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

class ArmMover:

    def __init__(self):
        rospy.init_node('moveit_service_node')

        robot = moveit_commander.RobotCommander()
        self._default_planning_time = 12
        self._group = moveit_commander.MoveGroupCommander("right_arm") 
        #self._group_left = moveit_commander.MoveGroupCommander("left_arm")
        self._s = rospy.Service('moveit_service', MoveArm, self.move_arm)
        

    def move_arm(self, req):

        plan = None
        success = False

        rospy.loginfo("Requested to move arm")
        if req.arm == "left":
            self._group = moveit_commander.MoveGroupCommander("left_arm")

        else:
            self._group = moveit_commander.MoveGroupCommander("right_arm")

        self._group.set_pose_target(req.goal)

        if req.planning_time > 0:
            self._group.set_planning_time(req.planning_time)
        else:
            self._group.set_planning_time(self._default_planning_time)
        plan = self._group.plan()
    
        success = self._group.go(wait=True)
           
        if plan.joint_trajectory.points and success:
            return MoveArmResponse(True)
        else:
            return MoveArmResponse(False)


if __name__ == "__main__":

    arm  = ArmMover()

    rospy.spin()
