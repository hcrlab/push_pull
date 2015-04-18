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
        self._default_planning_time = 7
        self._default_position_tolerance = 0.0001
        self._default_orientation_tolerance = 0.001
        self._group = moveit_commander.MoveGroupCommander("right_arm") 
        #self._group_left = moveit_commander.MoveGroupCommander("left_arm")
        self._s = rospy.Service('moveit_service', MoveArm, self.move_arm)

    def move_arm(self, req):

        plan = None
        success = False

        rospy.loginfo("Requested to move: " + req.group)
        self._group = moveit_commander.MoveGroupCommander(req.group)

        if req.position_tolerance > 0:
            self._group.set_goal_position_tolerance(req.position_tolerance)
        else:
            self._group.set_goal_position_tolerance(self._default_position_tolerance)

        if req.orientation_tolerance > 0:
            self._group.set_goal_orientation_tolerance(req.orientation_tolerance)
        else:
            self._group.set_goal_orientation_tolerance(self._default_orientation_tolerance)

        self._group.set_pose_target(req.goal)

        if req.planning_time > 0:
            self._group.set_planning_time(req.planning_time)
        else:
            self._group.set_planning_time(self._default_planning_time)

        plan = self._group.plan()
    
        success = self._group.go(wait=True)

        print "Plan: " + str(len(plan.joint_trajectory.points))
        print "Success: " + str(success)
           
        if plan.joint_trajectory.points and success:
            print "Moved arm successfully!"
            return MoveArmResponse(True)
        else:
            print "Failed to move arm"
            return MoveArmResponse(False)


if __name__ == "__main__":

    arm  = ArmMover()

    rospy.spin()
