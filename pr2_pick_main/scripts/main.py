#!/usr/bin/env python

'''The main state machine for the picking challenge. '''

from bin_data import BinData
from pr2_pick_manipulation.srv import DriveToPose
import argparse
import rospy
import smach
import smach_ros
from state_machine_factory import StateMachineBuilder
import states


def main(mock=False, test_move_to_bin=False, test_drop_off_item=False, debug=False,
         auto_reset=True):
    rospy.init_node('pr2_pick_state_machine')

    if test_move_to_bin:
        state_machine_type = StateMachineBuilder.TEST_MOVE_TO_BIN
    elif test_drop_off_item:
        state_machine_type = StateMachineBuilder.TEST_DROP_OFF_ITEM
    else:
        state_machine_type = StateMachineBuilder.DEFAULT

    sm = (
        StateMachineBuilder()
        .set_mock(mock)
        .set_state_machine(state_machine_type)
        .build()
    )

    # Whether to step through checkpoints.
    sm.userdata.debug = debug

    # The current bin being attempted.
    sm.userdata.current_bin = None

    # Holds data about the state of each bin.
    sm.userdata.bin_data = {}
    for bin_id in 'ABCDEFGHIJKL':
        sm.userdata.bin_data[bin_id] = BinData(id, False, False)

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
            except:
                pass
    if auto_reset:
        rospy.on_shutdown(on_shutdown)

    try:
        sis = smach_ros.IntrospectionServer(
            'state_machine_introspection_server', sm, '/')
        sis.start()
        outcome = sm.execute()
    except:
        sis.stop()
        rospy.signal_shutdown('Exception in the state machine.')

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        '--test_move_to_bin', action='store_true',
        help=('True to create a minimal state machine for testing the'
              'MoveToBin state.')
    )
    group.add_argument(
        '--test_drop_off_item', action='store_true',
        help=('True to create a minimal state machine for testing the'
              'DropOffItem state.')
    )

    parser.add_argument(
        '--mock', action='store_true',
        help=('True if you want to create a state machine with mock robot'
            ' components.')
    )
    parser.add_argument(
        '--debug', action='store_true',
        help=('True if you want to step through debugging checkpoints.')
    )
    parser.add_argument(
        '--auto_reset', action='store_true', default=True,
        help=('Set to true to make the robot to drive back to its starting'
              ' point when the state machine exits.')
    )
    args = parser.parse_args(args=rospy.myargv()[1:])
    sim_time = rospy.get_param('use_sim_time', False)
    if sim_time != False:
        rospy.logwarn('Warning: use_sim_time was set to true. Setting back to '
            'false. Verify your launch files.')
        rospy.set_param('use_sim_time', False)
    main(args.mock, args.test_move_to_bin, args.test_drop_off_item, args.debug,
         args.auto_reset)
