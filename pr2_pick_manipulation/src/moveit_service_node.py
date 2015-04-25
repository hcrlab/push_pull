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
        self._default_planning_time = 8
        self._default_position_tolerance = 0.0001
        self._default_orientation_tolerance = 0.001
        self._left_group = moveit_commander.MoveGroupCommander("left_arm")
        self._right_group = moveit_commander.MoveGroupCommander("right_arm") 
        self._s = rospy.Service('moveit_service', MoveArm, self.move_arm)
        self._ik_service = rospy.Service('move_arm_ik', MoveArmIk, self.move_arm_ik)

    def move_arm(self, req):

        plan = None
        success = False

        rospy.loginfo("Requested to move: " + req.group)

        while plan is None:
            try: 
                # TODO(jstn): maybe instead of creating a new commander every service call,
                # we can just create a commander for each move_group in __init__, and then
                # choose the appropriate one here?
                group = None
                if req.group == 'right_arm':
                    group = self._right_group
                elif req.group == 'left_arm':
                    group = self._left_group
                else:
                    rospy.logwarn('Calling moveit_service with untested group: {}'.format(req.group))
                    group = moveit_commander.MoveGroupCommander(req.group)
                #self._group = moveit_commander.MoveGroupCommander(req.group)

                if req.position_tolerance > 0:
                    group.set_goal_position_tolerance(req.position_tolerance)
                else:
                    group.set_goal_position_tolerance(self._default_position_tolerance)

                if req.orientation_tolerance > 0:
                    group.set_goal_orientation_tolerance(req.orientation_tolerance)
                else:
                    group.set_goal_orientation_tolerance(self._default_orientation_tolerance)

                group.set_pose_target(req.goal)

                if req.planning_time > 0:
                    group.set_planning_time(req.planning_time)
                else:
                    group.set_planning_time(self._default_planning_time)

                plan = group.plan()
            
                success = group.go(wait=True)

                print "Plan: " + str(len(plan.joint_trajectory.points))
                print "Success: " + str(success)
                   
                if plan.joint_trajectory.points and success:
                    print "Moved arm successfully!"
                    return MoveArmResponse(True)
                else:
                    print "Failed to move arm"
                    return MoveArmResponse(False)
            except RuntimeError:
                rospy.sleep(3.0)

    def move_arm_ik(self, request):
        group = None
        if request.arm == MoveArmIkRequest.LEFT_ARM:
            group = self._left_group
        else:
            group = self._right_group
        
        # TODO(jstn): TEST ################################
        group = moveit_commander.MoveGroupCommander('right_arm')
        request = MoveArmIkRequest()

        waypoints = [group.get_current_pose(), request.goal]
        rospy.loginfo(waypoints)
        group.compute_cartesian_path(waypoints, 0.01, 0, avoid_collisions=False)

        success = group.go(wait=True)
        return MoveArmIkResponse(success)


if __name__ == "__main__":

    arm  = ArmMover()

    rospy.spin()
