#!/usr/bin/env python

import rospy
import sys
from pr2_pick_manipulation.srv import *
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf


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
        self._ik_service = rospy.Service('move_arm_ik', MoveArmIk,
                                         self.move_arm_ik)
        self._tf = tf.TransformListener()

    def move_arm(self, req):
        plan = None
        success = False

        rospy.loginfo("Requested to move: " + req.group)

        while plan is None:
            try:
                group = None
                if req.group == 'right_arm':
                    group = self._right_group
                elif req.group == 'left_arm':
                    group = self._left_group
                else:
                    rospy.logwarn(
                        'Calling moveit_service with untested group: {}'.format(
                            req.group))
                    group = moveit_commander.MoveGroupCommander(req.group)

                if req.position_tolerance > 0:
                    group.set_goal_position_tolerance(req.position_tolerance)
                else:
                    group.set_goal_position_tolerance(
                        self._default_position_tolerance)

                if req.orientation_tolerance > 0:
                    group.set_goal_orientation_tolerance(
                        req.orientation_tolerance)
                else:
                    group.set_goal_orientation_tolerance(
                        self._default_orientation_tolerance)

                group.set_pose_target(req.goal)

                if req.planning_time > 0:
                    group.set_planning_time(req.planning_time)
                else:
                    group.set_planning_time(self._default_planning_time)

                plan = group.plan()

                if not req.plan_only:
                    success = group.go(wait=True)
                else:
                    if plan.joint_trajectory.points:
                        success = True

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
        """Moves the arm using MoveIt's Cartesian path planner.

        This moves the arm in a straight line to the goal pose. Returns True if
        the execution succeeded, False otherwise (e.g., the arm gets stuck).
        """
        group = None
        if request.arm == MoveArmIkRequest.LEFT_ARM:
            group = self._left_group
        else:
            group = self._right_group

        request.goal.header.stamp = rospy.Time(0)
        pose_in_planning_frame = None
        try:
            pose_in_planning_frame = self._tf.transformPose(
                group.get_planning_frame(), request.goal)
        except:
            rospy.logerr('[MoveArmIk]: No TF from {} to {}'.format(
                request.goal.header.frame_id, group.get_planning_frame()))
            return MoveArmIkResponse(False)

        waypoints = [pose_in_planning_frame.pose]
        trajectory, _ = group.compute_cartesian_path(waypoints, 0.01, 0,
                                                     avoid_collisions=False)

        group.clear_pose_targets()
        success = group.execute(trajectory)
        return MoveArmIkResponse(success)


if __name__ == "__main__":
    arm = ArmMover()
    rospy.spin()
