#!/usr/bin/env python

"""The main state machine for the picking challenge.
"""

from bin_data import BinData
import argparse
import rospy
import smach
import smach_ros
import state_machine_factory
import states


def main(mock=False, test_move_to_bin=False):
    rospy.init_node('pr2_pick_state_machine')
    sm = None
    if mock:
        sm = state_machine_factory.mock_robot()
    elif test_move_to_bin:
        sm = state_machine_factory.test_move_to_bin()
    else:
        sm = state_machine_factory.real_robot()

    # The current bin being attempted.
    sm.userdata.current_bin = None

    # Holds data about the state of each bin.
    sm.userdata.bin_data = {}
    for bin_id in 'ABCDEFGHIJKL':
        sm.userdata.bin_data[bin_id] = BinData(id, False, False)

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
        '--mock', action='store_true',
        help=('True if you want to create a state machine with mock robot'
            ' components.')
    )
    group.add_argument(
        '--test_move_to_bin', action='store_true',
        help=('True to create a minimal state machine for testing the'
              'MoveToBin state.')
    )
    args = parser.parse_args(args=rospy.myargv()[1:])
    main(args.mock, args.test_move_to_bin)
