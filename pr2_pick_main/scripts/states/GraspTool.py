from actionlib import SimpleActionClient
from copy import copy
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
import rospy
import smach
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import outcomes


class GraspTool(smach.State):
    ''' Grasps tool from the robot's right shoulder holster '''
    name = 'GRASP_TOOL'

    reach_for_tool_duration = rospy.Duration(5.0)
    joints = [
        'shoulder_pan_joint',
        'shoulder_lift_joint',
        'upper_arm_roll_joint',
        'elbow_flex_joint',
        'forearm_roll_joint',
        'wrist_flex_joint',
        'wrist_roll_joint',
    ]

    # hard coded position for grabbing the tool, assumes start position is
    # the result of untucking the arm
    waypoints = [
        [
            0.43892197578038483,  # shoulder_pan_joint,
            0.2224892543290122,   # shoulder_lift_joint,
            0.41629429054466693,  # upper_arm_roll_joint,
            -2.112756968769462,   # elbow_flex_joint,
            -6.56925891321456,    # forearm_roll_joint,
            -1.45717617700297,    # wrist_flex_joint,
            -1.094963706117333    # wrist_roll_joint,
        ],
    ]

    def __init__(self, **services):
        smach.State.__init__(
            self,
            outcomes=[
                outcomes.GRASP_TOOL_SUCCESS,
                outcomes.GRASP_TOOL_FAILURE,
            ],
            input_keys=['debug'],
        )
        self._tuck_arms = services['tuck_arms']
        self._set_grippers = services['set_grippers']

        self.arm_side = 'l'
        self.arm = SimpleActionClient(
            '{}_arm_controller/joint_trajectory_action'.format(self.arm_side),
            JointTrajectoryAction,
        )
        rospy.loginfo('Waiting for joint trajectory action server')
        self.arm.wait_for_server()

    def execute(self, userdata):
        self._tuck_arms(False, False)  # untuck arms
        self._set_grippers(True, False)  # open left gripper

        trajectory = JointTrajectoryGoal(
            trajectory=JointTrajectory(
                header=Header(
                    stamp=rospy.Time.now(),
                ),
                joint_names=[
                    '{}_{}'.format(self.arm_side, joint)
                    for joint in self.joints
                ],
                points=[
                    JointTrajectoryPoint(
                        positions=copy(waypoint),
                        velocities=[0.0 for component in waypoint],
                        time_from_start=self.reach_for_tool_duration
                    )
                    for waypoint in self.waypoints
                ]
            ),
        )

        rospy.loginfo(trajectory)
        if userdata.debug:
            raw_input('(Debug) Press enter to continue:')

        self.arm.send_goal(trajectory)
        result = self.arm.wait_for_result()

        self._set_grippers(False, False)  # close both grippers
        # TODO (Leah): Untuck left arm in a way that doesn't move right arm
        self._tuck_arms(False, False)  # untuck arms

        return outcomes.GRASP_TOOL_SUCCESS
