from pr2_pick_manipulation.srv import DriveAngular
from pr2_pick_manipulation.srv import DriveLinear
from pr2_pick_manipulation.srv import MoveArm
from pr2_pick_manipulation.srv import MoveHead
from pr2_pick_manipulation.srv import MoveTorso
from pr2_pick_manipulation.srv import SetGrippers
from pr2_pick_manipulation.srv import TuckArms
from pr2_pick_perception.msg import Object
from pr2_pick_perception.srv import CropShelf
from pr2_pick_perception.srv import CropShelfResponse
from pr2_pick_perception.srv import DeleteStaticTransform
from pr2_pick_perception.srv import LocalizeShelf
from pr2_pick_perception.srv import LocalizeShelfResponse
from pr2_pick_perception.srv import SetStaticTransform
from std_msgs.msg import String
from visualization_msgs.msg import Marker
import mock
import outcomes
import rospy
import smach
import states


def real_robot():
    ''' State machine builder for the real robot. '''
    services = real_robot_services()
    return build(**services)


def test_move_to_bin():
    '''
    Minimal state machine to test MoveToBin state, using real
    robot services
    '''
    all_services = real_robot_services()
    services = {
        key: all_services[key]
        for key in {
            'tts', 'tuck_arms', 'move_torso', 'set_grippers',
            'move_head', 'drive_linear', 'localize_object',
            'set_static_tf'
        }
    }
    return build_for_move_to_bin(**services)


def real_robot_services():
    return {
        'tts': rospy.Publisher('/festival_tts', String),
        'tuck_arms': rospy.ServiceProxy('tuck_arms_service', TuckArms),
        'move_torso': rospy.ServiceProxy('torso_service', MoveTorso),
        'set_grippers': rospy.ServiceProxy('gripper_service', SetGrippers),
        'move_head': rospy.ServiceProxy('move_head_service', MoveHead),
        'moveit_move_arm': rospy.ServiceProxy('moveit_service', MoveArm),
        'localize_object': rospy.ServiceProxy('perception/localize_object',
                                              LocalizeShelf),
        'set_static_tf': rospy.ServiceProxy('perception/set_static_transform',
                                            SetStaticTransform),
        'drive_linear': rospy.ServiceProxy('drive_linear_service', DriveLinear),
        'drive_angular': rospy.ServiceProxy('drive_angular_service', DriveAngular),
        'markers': rospy.Publisher('pr2_pick_visualization', Marker),
        'crop_shelf': rospy.ServiceProxy('shelf_cropper', CropShelf)
     }


def side_effect(name, return_value=True):
    """A side effect for mock functions.

    Causes all wrapped functions to return True, and logs their arguments.
    """
    def wrapped(*args, **kwargs):
        rospy.loginfo('Calling {}{}'.format(name, args))
        return return_value
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

    moveit_move_arm = rospy.ServiceProxy('moveit_service', MoveArm)
    moveit_move_arm.wait_for_service = mock.Mock(return_value=None)
    moveit_move_arm.call = mock.Mock(
        side_effect=side_effect('moveit_move_arm'))

    shelf_response = LocalizeShelfResponse()
    shelf_obj = Object()
    shelf_obj.header.frame_id = 'odom_combined'
    shelf_response.locations.objects.append(shelf_obj)
    localize_object = rospy.ServiceProxy('perception/localize_object',
                                         LocalizeShelf)
    localize_object.wait_for_service = mock.Mock(return_value=None)
    localize_object.call = mock.Mock(
        side_effect=side_effect('localize_object',
                                return_value=shelf_response))

    set_static_tf = rospy.ServiceProxy('perception/set_static_transform',
                                        SetStaticTransform)
    set_static_tf.wait_for_service = mock.Mock(return_value=None)
    set_static_tf.call = mock.Mock(
        side_effect=side_effect('set_static_tf'))

    drive_linear = rospy.ServiceProxy('drive_linear_service', DriveLinear)
    drive_linear.wait_for_service = mock.Mock(return_value=None)
    drive_linear.call = mock.Mock(side_effect=side_effect('drive_linear'))

    drive_angular = rospy.ServiceProxy('drive_angular_service', DriveAngular)
    drive_angular.wait_for_service = mock.Mock(return_value=None)
    drive_angular.call = mock.Mock(side_effect=side_effect('drive_angular'))

    markers = rospy.Publisher('pr2_pick_visualization', Marker)
    markers.publish = mock.Mock(side_effect=side_effect('markers'))

    crop_response = CropShelfResponse()
    crop_shelf = rospy.ServiceProxy('shelf_cropper', CropShelf)
    crop_shelf.wait_for_service = mock.Mock(return_value=None)
    crop_shelf.call = mock.Mock(
        side_effect=side_effect('shelf_cropper', return_value=crop_response))

    return build(tts, tuck_arms, move_torso, set_grippers, move_head,
                 moveit_move_arm, localize_object, set_static_tf, drive_linear,
                 drive_angular, markers, crop_shelf)


def build(tts, tuck_arms, move_torso, set_grippers, move_head, moveit_move_arm,
          localize_object, set_static_tf, drive_linear, drive_angular, markers,
          crop_shelf):
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
            states.FindShelf(tts, localize_object, set_static_tf, markers),
            transitions={
                outcomes.FIND_SHELF_SUCCESS: states.UpdatePlan.name,
                outcomes.FIND_SHELF_FAILURE: outcomes.CHALLENGE_FAILURE
            },
            remapping={
                'debug': 'debug'
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
            states.MoveToBin(tts, drive_linear, drive_angular, move_torso, markers),
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
            states.SenseBin(tts, crop_shelf),
            transitions={
                outcomes.SENSE_BIN_SUCCESS: states.Grasp.name,
                outcomes.SENSE_BIN_NO_OBJECTS: states.UpdatePlan.name,
                outcomes.SENSE_BIN_FAILURE: outcomes.CHALLENGE_FAILURE
            },
            remapping={
                'bin_id': 'current_bin',
                'clusters': 'clusters'
            }
        )
        smach.StateMachine.add(
            states.Grasp.name,
            states.Grasp(tts, set_grippers, tuck_arms, moveit_move_arm),
            transitions={
                outcomes.GRASP_SUCCESS: states.ExtractItem.name,
                outcomes.GRASP_FAILURE: (
                    outcomes.CHALLENGE_FAILURE
                )
            },
            remapping={
                'bin_id': 'current_bin',
                'clusters': 'clusters'
            }
        )
        smach.StateMachine.add(
            states.ExtractItem.name,
            states.ExtractItem(tts, moveit_move_arm),
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
            states.DropOffItem(tts),
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


def build_for_move_to_bin(tts, tuck_arms, move_torso, drive_linear,
                          set_grippers, move_head, localize_object,
                          set_static_tf):
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
            states.FindShelf(localize_object, set_static_tf),
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
            states.MoveToBin(drive_linear, move_torso),
            transitions={
                outcomes.MOVE_TO_BIN_SUCCESS: states.UpdatePlan.name,
                outcomes.MOVE_TO_BIN_FAILURE: outcomes.CHALLENGE_FAILURE
            },
            remapping={
                'bin_id': 'current_bin'
            }
        )
    return sm
