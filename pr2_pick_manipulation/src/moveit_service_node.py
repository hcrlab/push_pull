#!/usr/bin/env python

from actionlib import SimpleActionClient
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from pr2_pick_manipulation.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
import rospy
import sys
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
        self._compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
        self._r_joint_traj_action = SimpleActionClient(
            'r_arm_controller/joint_trajectory_action', JointTrajectoryAction)
        self._l_joint_traj_action = SimpleActionClient(
            'l_arm_controller/joint_trajectory_action', JointTrajectoryAction)

    def move_arm(self, req):
        plan = None
        success = False

        rospy.loginfo("Requested to move: " + req.group)

        self._compute_ik.wait_for_service()

        rospy.loginfo("Moveit compute ik service available")
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
        """Moves the arm using IK and JointTrajectoryAction.
        """
        group = None
        group_name = ''
        traj_action = None
        arm_prefix = ''
        robot_joint_names = []
        if request.arm == MoveArmIkRequest.LEFT_ARM:
            group = self._left_group
            group_name = 'left_arm'
            traj_action = self._l_joint_traj_action
            arm_prefix = 'l'
            robot_joint_names = rospy.get_param('/l_arm_controller/joints')
        else:
            group = self._right_group
            group_name = 'right_arm'
            traj_action = self._r_joint_traj_action
            arm_prefix = 'r'
            robot_joint_names = rospy.get_param('/r_arm_controller/joints')

        service_request = GetPositionIKRequest()
        service_request.ik_request.group_name = group_name
        service_request.ik_request.pose_stamped = request.goal
        service_request.ik_request.pose_stamped.header.stamp = rospy.Time(0)
        service_request.ik_request.avoid_collisions = False
        self._compute_ik.wait_for_service()
        service_response = self._compute_ik(service_request)
        if service_response.error_code.val != moveit_msgs.msg.MoveItErrorCodes.SUCCESS:
            return MoveArmIkResponse(False)

        joint_names = service_response.solution.joint_state.name
        joint_positions = service_response.solution.joint_state.position
        filtered_joint_names = []
        filtered_joint_positions = []
        for name, position in zip(joint_names, joint_positions):
            if name in robot_joint_names:
                filtered_joint_names.append(name)
                filtered_joint_positions.append(position)

        duration = request.duration
        if duration.to_sec() <= 0:
            duration = rospy.Duration(5)

        goal = JointTrajectoryGoal()
        goal.trajectory.header.stamp = rospy.Time.now()
        goal.trajectory.joint_names = filtered_joint_names
        goal.trajectory.points = [
            JointTrajectoryPoint(positions=filtered_joint_positions,
                                 time_from_start=duration)
        ]
        traj_action.send_goal(goal)
        traj_action.wait_for_result()

        success = True
        return MoveArmIkResponse(success)

    def move_arm_cart(self, request):
        """Moves the arm using MoveIt's Cartesian path planner.

        DEPRECATED: No service actually uses this function.

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
