from pr2_pick_manipulation.srv import MoveTorso, MoveTorsoRequest
from pr2_pick_manipulation.srv import SetGrippers
from pr2_pick_manipulation.srv import TuckArms
from pr2_pick_manipulation.srv import MoveHead
from std_msgs.msg import String
import mock
import outcomes
import rospy
import smach
import states


def real_robot():
    """State machine builder for the real robot.
    """
    tts = rospy.Publisher('/festival_tts', String)
    tuck_arms = rospy.ServiceProxy('tuck_arms_service', TuckArms)
    move_torso = rospy.ServiceProxy('torso_service', MoveTorso)
    set_grippers = rospy.ServiceProxy('gripper_service', SetGrippers)
    move_head = rospy.ServiceProxy('move_head_service', MoveHead)
    return build(tts, tuck_arms, move_torso, set_grippers, move_head)


def side_effect(name):
    """A side effect for mock functions.

    Causes all wrapped functions to return True, and logs their arguments.
    """
    def wrapped(*args, **kwargs):
        rospy.loginfo('Calling {}{}'.format(name, args))
        return True
    return wrapped


def mock_robot():
    """Mock robot state machine builder.

    This will cause all services and publishers to do nothing. Their arguments
    will be printed to the screen, and all service calls will succeed.  This is
    useful for when the robot is being used by someone else, but you want to
    run the state machine and test the logic of your code at the same time.

    To change the behavior for a particular state, you can just instantiate
    real publishers or services for the state you're testing.
    """
    tts = rospy.Publisher('/festival_tts', String)
    tts.publish = mock.Mock(side_effect=side_effect('tts'))

    tuck_arms = rospy.ServiceProxy('tuck_arms_service', TuckArms)
    tuck_arms.wait_for_service = mock.Mock(return_value=None)
    tuck_arms.call = mock.Mock(side_effect=side_effect('tuck_arms'))

    move_torso = rospy.ServiceProxy('torso_service', MoveTorso)
    move_torso.wait_for_service = mock.Mock(return_value=None)
    move_torso.call = mock.Mock(side_effect=side_effect('move_torso'))

    set_grippers = rospy.ServiceProxy('gripper_service', SetGrippers)
    set_grippers.wait_for_service = mock.Mock(return_value=None)
    set_grippers.call = mock.Mock(side_effect=side_effect('set_grippers'))

    move_head = rospy.ServiceProxy('move_head_service', MoveHead)
    move_head.wait_for_service = mock.Mock(return_value=None)
    move_head.call = mock.Mock(side_effect=side_effect('move_head'))
    return build(tts, tuck_arms, move_torso, set_grippers, move_head)


def build(tts, tuck_arms, move_torso, set_grippers, move_head):
    """Builds the main state machine.

    You probably want to call either real_robot() or mock_robot() to build a
    state machine instead of this method.

    Args:
      tts: A text-to-speech publisher.
      tuck_arms: The tuck arms service proxy.
      move_torso: The torso service proxy.
      set_grippers: The grippers service proxy.
      move_head: The head service proxy.
    """
    sm = smach.StateMachine(outcomes=[
        outcomes.CHALLENGE_SUCCESS,
        outcomes.CHALLENGE_FAILURE
    ])
    with sm:
        smach.StateMachine.add(
            states.StartPose.name,
            states.StartPose(tts, tuck_arms, move_torso, set_grippers,
                             move_head),
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
            states.UpdatePlan(tts),
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
            states.Grasp(set_grippers, tuck_arms),
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
                'bin_data': 'bin_data',
                'output_bin_data': 'bin_data',
            }
        )
    return sm
