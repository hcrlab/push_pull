#!/usr/bin/env python

"""The main state machine for the picking challenge.
"""

import outcomes
import rospy
import smach
import smach_ros
import states


def main():
    rospy.init_node('pr2_pick_state_machine')

    sm = smach.StateMachine(outcomes=[
        outcomes.CHALLENGE_SUCCESS,
        outcomes.CHALLENGE_FAILURE
    ])

    # The set of bins we have already attempted to pick from.
    sm.userdata.last_bin_id = None

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
            states.ReadContestFile.name,
            states.ReadContestFile(),
            transitions={
                outcomes.READ_CONTEST_SUCCESS: states.FindShelf.name,
                outcomes.READ_CONTEST_FAILURE: outcomes.CHALLENGE_FAILURE
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
            }
        )
        smach.StateMachine.add(
            states.MoveToBin.name,
            states.MoveToBin(),
            transitions={
                outcomes.MOVE_TO_BIN_SUCCESS: states.SenseBin.name,
                outcomes.MOVE_TO_BIN_FAILURE: outcomes.CHALLENGE_FAILURE
            }
        )
        smach.StateMachine.add(
            states.SenseBin.name,
            states.SenseBin(),
            transitions={
                outcomes.SENSE_BIN_SUCCESS: states.Grasp.name,
                outcomes.SENSE_BIN_NO_OBJECTS: states.UpdatePlan.name,
                outcomes.SENSE_BIN_FAILURE: outcomes.CHALLENGE_FAILURE
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
            }
        )
        smach.StateMachine.add(
            states.ExtractItem.name,
            states.ExtractItem(),
            transitions={
                outcomes.EXTRACT_ITEM_SUCCESS: states.DropOffItem.name,
                outcomes.EXTRACT_ITEM_FAILURE: states.UpdatePlan.name
            }
        )
        smach.StateMachine.add(
            states.DropOffItem.name,
            states.DropOffItem(),
            transitions={
                outcomes.DROP_OFF_ITEM_SUCCESS: states.UpdatePlan.name,
                outcomes.DROP_OFF_ITEM_FAILURE: states.UpdatePlan.name
            }
        )

    sis = smach_ros.IntrospectionServer('state_machine_introspection_server',
                                        sm,
                                        '/')
    sis.start()
    outcome = sm.execute()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
