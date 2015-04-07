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
                outcomes.UPDATE_PLAN_NEXT_OBJECT: states.PrepareSensing.name,
                outcomes.UPDATE_PLAN_NO_MORE_OBJECTS: outcomes.CHALLENGE_SUCCESS,
                outcomes.UPDATE_PLAN_FAILURE: outcomes.CHALLENGE_FAILURE
            }
        )
        smach.StateMachine.add(
            states.PrepareSensing.name,
            states.PrepareSensing(),
            transitions={
                outcomes.PREPARE_SENSING_SUCCESS: states.SenseBin.name,
                outcomes.PREPARE_SENSING_FAILURE: outcomes.CHALLENGE_FAILURE
            }
        )
        smach.StateMachine.add(
            states.SenseBin.name,
            states.SenseBin(),
            transitions={
                outcomes.SENSE_BIN_SUCCESS: states.PrepareManipulation.name,
                outcomes.SENSE_BIN_NO_OBJECTS: states.UpdatePlan.name,
                outcomes.SENSE_BIN_FAILURE: outcomes.CHALLENGE_FAILURE
            }
        )
        smach.StateMachine.add(
            states.PrepareManipulation.name,
            states.PrepareManipulation(),
            transitions={
                outcomes.PREPARE_MANIPULATION_SUCCESS: states.ManipulateObject.name,
                outcomes.PREPARE_MANIPULATION_FAILURE: (
                    outcomes.CHALLENGE_FAILURE
                )
            }
        )
        smach.StateMachine.add(
            states.ManipulateObject.name,
            states.ManipulateObject(),
            transitions={
                outcomes.MANIPULATE_OBJECT_SUCCESS: states.DropOffObject.name,
                outcomes.MANIPULATE_OBJECT_FAILURE: states.UpdatePlan.name
            }
        )
        smach.StateMachine.add(
            states.DropOffObject.name,
            states.DropOffObject(),
            transitions={
                outcomes.DROP_OFF_OBJECT_SUCCESS: states.UpdatePlan.name,
                outcomes.DROP_OFF_OBJECT_FAILURE: states.UpdatePlan.name
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
