#!/usr/bin/env python
'''The main state machine for the picking challenge. '''

from bin_data import BinData
from pr2_pick_manipulation.srv import DriveToPose
from pr2_pick_manipulation.srv import SetGrippers
from pr2_pick_manipulation.srv import MoveTorso, MoveTorsoRequest
import argparse
import outcomes
import rospy
import smach
import smach_ros
from state_machine_factory import StateMachineBuilder
import states

def main(plan_grasp = False,
         test_grasp_tool=False,
         debug=False,
         auto_reset=True,
         attempts_per_bin=2):
    rospy.init_node('pr2_pick_state_machine')

    if test_grasp_tool:
        state_machine_type = StateMachineBuilder.TEST_GRASP_TOOL
    elif plan_grasp:
        state_machine_type = StateMachineBuilder.PLAN_GRASP
    else:
        state_machine_type = StateMachineBuilder.PLAN_GRASP

    sm = (StateMachineBuilder()
          .set_state_machine(state_machine_type).build())

    # Whether to step through checkpoints.
    sm.userdata.debug = debug

    # The current bin being attempted.
    sm.userdata.current_bin = None

    # Holds data about the state of each bin.
    sm.userdata.bin_data = {}
    for bin_id in 'ABCDEFGHIJKL':
        num_attempts = attempts_per_bin
        if bin_id in 'ABC':
            num_attempts = 1
        sm.userdata.bin_data[bin_id] = BinData(id, False, False,
                                               num_attempts)

    # The starting pose of the robot, in odom_combined.
    sm.userdata.start_pose = None

    # A list of clusters (pr2_pick_perception/Cluster.msg) for objects in the
    # current bin.
    sm.userdata.clusters = []

    def on_shutdown():
        if sm.userdata is not None:
            try:
                drive_to_pose = rospy.ServiceProxy('drive_to_pose_service',
                                                   DriveToPose)
                drive_to_pose(pose=sm.userdata.start_pose,
                              linearVelocity=0.1,
                              angularVelocity=0.1)
                set_grippers = rospy.ServiceProxy('set_grippers_service',
                                                  SetGrippers)
                set_grippers.wait_for_service()
                set_grippers(True, True, -1)

            except:
                pass

    if auto_reset:
        rospy.on_shutdown(on_shutdown)

    # Set torso height low initially.
    try:
        move_torso = rospy.ServiceProxy('torso_service', MoveTorso)
        move_torso.wait_for_service()
        torso_success = move_torso(MoveTorsoRequest.MIN_HEIGHT)
    except:
        pass

    try:
        sis = smach_ros.IntrospectionServer(
            'state_machine_introspection_server', sm, '/')
        sis.start()
        outcome = sm.execute()
        if outcome == outcomes.CHALLENGE_FAILURE:
            rospy.signal_shutdown('Challenge failed.')
    except:
        sis.stop()
        rospy.signal_shutdown('Exception in the state machine.')

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        '--test_drop_off_item',
        action='store_true',
        help=('True to create a minimal state machine for testing the'
              'DropOffItem state.'))
    group.add_argument(
        '--test_grasp_tool',
        action='store_true',
        help=('True to create a minimal state machine for testing the'
              'GraspTool state.'))
    parser.add_argument(
        '--debug',
        action='store_true',
        help=('True if you want to step through debugging checkpoints.'))
    parser.add_argument(
        '--auto_reset',
        action='store_true',
        default=True,
        help=('Set to true to make the robot to drive back to its starting'
              ' point when the state machine exits.'))
    parser.add_argument(
        '--attempts_per_bin',
        default='2',
        type=int,
        help=('Number of times to attempt to grasp each item before'
              ' giving up.'))
    group.add_argument(
        '--plan_grasp',
        action='store_true',
        help=('True to test grasp planning.'))

    args = parser.parse_args(args=rospy.myargv()[1:])
    sim_time = rospy.get_param('use_sim_time', False)
    if sim_time != False:
        rospy.logwarn('Warning: use_sim_time was set to true. Setting back to '
                      'false. Verify your launch files.')
        rospy.set_param('use_sim_time', False)
    main(args.plan_grasp, args.test_grasp_tool,args.debug, args.auto_reset,
         args.attempts_per_bin)
