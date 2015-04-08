#!/usr/bin/env python

"""The main state machine for the picking challenge.
"""

from collections import namedtuple
import outcomes
import rospy
import smach
import smach_ros
import states


# Bin data structure.
#   id: string, 'A' - 'L' according to the diagram on the contest website.
#   visited: boolean, true if the robot has tried to pick an item from this bin
#     at least once, false otherwise.
#   succeeded: boolean, true if the robot successfully picked an item from this
#     bin, false otherwise. succeeded = false either means that there was an
#     unsuccessful attempt, or it hasn't been tried yet, check visited to
#     distinguish between these two cases.
BinData = namedtuple('BinData', ['id', 'visited', 'succeeded'])


def main():
    rospy.init_node('pr2_pick_state_machine')

    sm = smach.StateMachine(outcomes=[
        outcomes.CHALLENGE_SUCCESS,
        outcomes.CHALLENGE_FAILURE
    ])

    # The current bin being attempted.
    sm.userdata.current_bin = None

    # Holds data about the state of each bin.
    sm.userdata.bin_data = {}
    for bin_id in 'ABCDEFGHIJKL':
        sm.userdata.bin_data[bin_id] = BinData(id, False, False)

    with sm:
        smach.StateMachine.add(
            states.StartPose.name,
            states.StartPose(),
            transitions={
                outcomes.START_POSE_SUCCESS: states.FindShelf.name,
                outcomes.START_POSE_FAILURE: outcomes.CHALLENGE_FAILURE
            }
        )
        smach.StateMachine.add(
            states.FindShelf.name,
            states.FindShelf(),
            transitions={
                outcomes.FIND_SHELF_SUCCESS: states.UpdatePlan.name,
                outcomes.FIND_SHELF_FAILURE: outcomes.CHALLENGE_FAILURE
            }
        )
        smach.StateMachine.add(
            states.UpdatePlan.name,
            states.UpdatePlan(),
            transitions={
                outcomes.UPDATE_PLAN_NEXT_OBJECT: states.MoveToBin.name,
                outcomes.UPDATE_PLAN_NO_MORE_OBJECTS: outcomes.CHALLENGE_SUCCESS,
                outcomes.UPDATE_PLAN_FAILURE: outcomes.CHALLENGE_FAILURE
            },
            remapping={
                'bin_data': 'bin_data',
                'output_bin_data': 'bin_data',
                'next_bin': 'current_bin'
            }
        )
        smach.StateMachine.add(
            states.MoveToBin.name,
            states.MoveToBin(),
            transitions={
                outcomes.MOVE_TO_BIN_SUCCESS: states.SenseBin.name,
                outcomes.MOVE_TO_BIN_FAILURE: outcomes.CHALLENGE_FAILURE
            },
            remapping={
                'bin_id': 'current_bin'
            }
        )
        smach.StateMachine.add(
            states.SenseBin.name,
            states.SenseBin(),
            transitions={
                outcomes.SENSE_BIN_SUCCESS: states.Grasp.name,
                outcomes.SENSE_BIN_NO_OBJECTS: states.UpdatePlan.name,
                outcomes.SENSE_BIN_FAILURE: outcomes.CHALLENGE_FAILURE
            },
            remapping={
                'bin_id': 'current_bin'
            }
        )
        smach.StateMachine.add(
            states.Grasp.name,
            states.Grasp(),
            transitions={
                outcomes.GRASP_SUCCESS: states.ExtractItem.name,
                outcomes.GRASP_FAILURE: (
                    outcomes.CHALLENGE_FAILURE
                )
            },
            remapping={
                'bin_id': 'current_bin'
            }
        )
        smach.StateMachine.add(
            states.ExtractItem.name,
            states.ExtractItem(),
            transitions={
                outcomes.EXTRACT_ITEM_SUCCESS: states.DropOffItem.name,
                outcomes.EXTRACT_ITEM_FAILURE: states.UpdatePlan.name
            },
            remapping={
                'bin_id': 'current_bin'
            }
        )
        smach.StateMachine.add(
            states.DropOffItem.name,
            states.DropOffItem(),
            transitions={
                outcomes.DROP_OFF_ITEM_SUCCESS: states.UpdatePlan.name,
                outcomes.DROP_OFF_ITEM_FAILURE: states.UpdatePlan.name
            },
            remapping={
                'bin_id': 'current_bin',
                'bin_data': 'bin_data'
            }
        )

        try:
            sis = smach_ros.IntrospectionServer('state_machine_introspection_server',
                                                sm,
                                                '/')
            sis.start()
            outcome = sm.execute()
        except:
            sis.stop()
            rospy.signal_shutdown('Exception in the state machine.')

        rospy.spin()
        sis.stop()


if __name__ == '__main__':
    main()
