from interactive_markers.interactive_marker_server import InteractiveMarkerServer
import mock
import rospy
from std_msgs.msg import String
import smach
import tf
from visualization_msgs.msg import Marker

import outcomes
from pr2_pick_manipulation.srv import DriveAngular, DriveLinear, \
    DriveToPose, GetPose, MoveArm, MoveHead, MoveTorso, SetGrippers, \
    TuckArms
from pr2_pick_perception.msg import Object
from pr2_pick_perception.srv import CropShelf, CropShelfResponse, \
    DeleteStaticTransform, FindCentroid, LocalizeShelf, LocalizeShelfResponse, \
    SetStaticTransform
import states
import visualization


class StateMachineBuilder(object):

    # slugs to represent different state machines
    TEST_MOVE_TO_BIN = 'test-move-to-bin'
    TEST_DROP_OFF_ITEM = 'test-drop-off-item'
    DEFAULT = 'default'

    def __init__(self):
        self.mock = False
        self.state_machine_identifier = StateMachineBuilder.DEFAULT

    def set_mock(self, mock):
        ''' Sets mock status. True to use mock services, false to use real services. '''
        self.mock = mock
        return self

    def set_state_machine(self, state_machine_identifier):
        ''' Sets which state machine to use. '''
        self.state_machine_identifier = state_machine_identifier
        return self

    def build(self):
        ''' Build the state machine with previously specified mock status and
        state machine type. '''
        services = None
        if self.mock:
            services = self.mock_robot_services()
        else:
            services = self.real_robot_services()

        if self.state_machine_identifier == StateMachineBuilder.TEST_MOVE_TO_BIN:
            build = self.build_sm_for_move_to_bin
        elif self.state_machine_identifier == StateMachineBuilder.TEST_DROP_OFF_ITEM:
            build = self.build_sm_for_drop_off_item
        else:
            build = self.build_sm

        return build(**services)

    def real_robot_services(self):
        return {
            # Speech
            'tts': rospy.Publisher('/festival_tts', String),

            # Manipulation
            'drive_angular': rospy.ServiceProxy('drive_angular_service', DriveAngular),
            'drive_linear': rospy.ServiceProxy('drive_linear_service', DriveLinear),
            'drive_to_pose': rospy.ServiceProxy('drive_to_pose_service', DriveToPose),
            'move_torso': rospy.ServiceProxy('torso_service', MoveTorso),
            'move_head': rospy.ServiceProxy('move_head_service', MoveHead),
            'moveit_move_arm': rospy.ServiceProxy('moveit_service', MoveArm),
            'set_grippers': rospy.ServiceProxy('gripper_service', SetGrippers),
            'tuck_arms': rospy.ServiceProxy('tuck_arms_service', TuckArms),

            # World and Perception
            'crop_shelf': rospy.ServiceProxy('perception/shelf_cropper', CropShelf),
            'find_centroid': rospy.ServiceProxy('perception/find_centroid', FindCentroid),
            'interactive_marker_server': InteractiveMarkerServer('pr2_pick_interactive_markers'),
            'localize_object': rospy.ServiceProxy('perception/localize_object',
                                                  LocalizeShelf),
            'markers': rospy.Publisher('pr2_pick_visualization', Marker),
            'set_static_tf': rospy.ServiceProxy('perception/set_static_transform',
                                                SetStaticTransform),
            'tf_listener': tf.TransformListener(),
         }

    def side_effect(self, name, return_value=True):
        '''A side effect for mock functions.

        Causes all wrapped functions to return True, and logs their arguments.
        '''
        def wrapped(*args, **kwargs):
            rospy.loginfo('Calling {}{}'.format(name, args))
            return return_value
        return wrapped

    def mock_robot_services(self):
        '''Mock robot service builder.

        This will cause all services and publishers to do nothing. Their arguments
        will be printed to the screen, and all service calls will succeed.  This is
        useful for when the robot is being used by someone else, but you want to
        run the state machine and test the logic of your code at the same time.

        To change the behavior for a particular state, you can just instantiate
        real publishers or services for the state you're testing.
        '''
        tts = rospy.Publisher('/festival_tts', String)
        tts.publish = mock.Mock(side_effect=self.side_effect('tts'))

        tuck_arms = rospy.ServiceProxy('tuck_arms_service', TuckArms)
        tuck_arms.wait_for_service = mock.Mock(return_value=None)
        tuck_arms.call = mock.Mock(side_effect=self.side_effect('tuck_arms'))

        move_torso = rospy.ServiceProxy('torso_service', MoveTorso)
        move_torso.wait_for_service = mock.Mock(return_value=None)
        move_torso.call = mock.Mock(side_effect=self.side_effect('move_torso'))

        set_grippers = rospy.ServiceProxy('gripper_service', SetGrippers)
        set_grippers.wait_for_service = mock.Mock(return_value=None)
        set_grippers.call = mock.Mock(side_effect=self.side_effect('set_grippers'))

        move_head = rospy.ServiceProxy('move_head_service', MoveHead)
        move_head.wait_for_service = mock.Mock(return_value=None)
        move_head.call = mock.Mock(side_effect=self.side_effect('move_head'))

        moveit_move_arm = rospy.ServiceProxy('moveit_service', MoveArm)
        moveit_move_arm.wait_for_service = mock.Mock(return_value=None)
        moveit_move_arm.call = mock.Mock(
            side_effect=self.side_effect('moveit_move_arm'))

        shelf_response = LocalizeShelfResponse()
        shelf_obj = Object()
        shelf_obj.header.frame_id = 'odom_combined'
        shelf_response.locations.objects.append(shelf_obj)
        localize_object = rospy.ServiceProxy('perception/localize_object',
                                             LocalizeShelf)
        localize_object.wait_for_service = mock.Mock(return_value=None)
        localize_object.call = mock.Mock(
            side_effect=self.side_effect('localize_object',
                                    return_value=shelf_response))

        set_static_tf = rospy.ServiceProxy('perception/set_static_transform',
                                            SetStaticTransform)
        set_static_tf.wait_for_service = mock.Mock(return_value=None)
        set_static_tf.call = mock.Mock(
            side_effect=self.side_effect('set_static_tf'))

        drive_linear = rospy.ServiceProxy('drive_linear_service', DriveLinear)
        drive_linear.wait_for_service = mock.Mock(return_value=None)
        drive_linear.call = mock.Mock(side_effect=self.side_effect('drive_linear'))

        drive_angular = rospy.ServiceProxy('drive_angular_service', DriveAngular)
        drive_angular.wait_for_service = mock.Mock(return_value=None)
        drive_angular.call = mock.Mock(side_effect=self.side_effect('drive_angular'))

        markers = rospy.Publisher('pr2_pick_visualization', Marker)
        markers.publish = mock.Mock(side_effect=self.side_effect('markers'))

        crop_response = CropShelfResponse()
        crop_shelf = rospy.ServiceProxy('perception/shelf_cropper', CropShelf)
        crop_shelf.wait_for_service = mock.Mock(return_value=None)
        crop_shelf.call = mock.Mock(
            side_effect=self.side_effect('shelf_cropper', return_value=crop_response))

        drive_to_pose = rospy.ServiceProxy('drive_to_pose_service', DriveToPose)
        drive_to_pose.wait_for_service = mock.Mock(return_value=None)
        drive_to_pose.call = mock.Mock(side_effect=self.side_effect('drive_to_pose'))

        interactive_marker_server = InteractiveMarkerServer('pr2_pick_interactive_markers')
        interactive_marker.insert = mock.Mock(side_effect=self.side_effect('server.insert'))
        interactive_marker.applyChanges = mock.Mock(side_effect=self.side_effect('server.applyChanges'))

        return {
            # Speech
            'tts': tts,

            # Manipulation
            'drive_angular': drive_angular,
            'drive_linear': drive_linear,
            'drive_to_pose': drive_to_pose,
            'move_torso': move_torso,
            'move_head': move_head,
            'moveit_move_arm': moveit_move_arm,
            'set_grippers': set_grippers,
            'tuck_arms': tuck_arms,

            # World and Perception
            'crop_shelf': crop_shelf,
            'interactive_marker_server': interactive_marker_server,
            'localize_object': localize_object,
            'markers': markers,
            'set_static_tf': set_static_tf,
         }

    def build_sm_for_move_to_bin(self, **services):
        sm = smach.StateMachine(outcomes=[
            outcomes.CHALLENGE_SUCCESS,
            outcomes.CHALLENGE_FAILURE
        ])

        def mock_update_plan_execute(userdata):
            ''' Ask the user which bin to go to next. Don't worry about updating bin_data. '''
            default_next_bin = 'A'
            input_bin = raw_input('(Debug) Enter the next bin [{}]:'.format(default_next_bin))
            if len(input_bin) > 0 and input_bin[0] in 'ABCDEFGHIJKL':
                bin_letter = input_bin[0]
            else:
                bin_letter = default_next_bin

            userdata.next_bin = bin_letter
            return outcomes.UPDATE_PLAN_NEXT_OBJECT

        mock_update_plan = states.UpdatePlan(**services)
        mock_update_plan.execute = mock_update_plan_execute

        with sm:
            smach.StateMachine.add(
                states.StartPose.name,
                states.StartPose(**services),
                transitions={
                    outcomes.START_POSE_SUCCESS: states.FindShelf.name,
                    outcomes.START_POSE_FAILURE: outcomes.CHALLENGE_FAILURE
                }
            )
            smach.StateMachine.add(
                states.FindShelf.name,
                states.FindShelf(**services),
                transitions={
                    outcomes.FIND_SHELF_SUCCESS: states.UpdatePlan.name,
                    outcomes.FIND_SHELF_FAILURE: outcomes.CHALLENGE_FAILURE
                }
            )
            smach.StateMachine.add(
                states.UpdatePlan.name,
                mock_update_plan,
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
                states.MoveToBin(**services),
                transitions={
                    outcomes.MOVE_TO_BIN_SUCCESS: states.UpdatePlan.name,
                    outcomes.MOVE_TO_BIN_FAILURE: outcomes.CHALLENGE_FAILURE
                },
                remapping={
                    'bin_id': 'current_bin'
                }
            )
        return sm

    def build_sm_for_drop_off_item(self, **services):
        sm = smach.StateMachine(outcomes=[
            outcomes.CHALLENGE_SUCCESS,
            outcomes.CHALLENGE_FAILURE
        ])
        with sm:
            smach.StateMachine.add(
                states.StartPose.name,
                states.StartPose(**services),
                transitions={
                    outcomes.START_POSE_SUCCESS: states.FindShelf.name,
                    outcomes.START_POSE_FAILURE: outcomes.CHALLENGE_FAILURE
                }
            )
            smach.StateMachine.add(
                states.FindShelf.name,
                states.FindShelf(**services),
                transitions={
                    outcomes.FIND_SHELF_SUCCESS: states.DropOffItem.name,
                    outcomes.FIND_SHELF_FAILURE: outcomes.CHALLENGE_FAILURE
                }
            )
            smach.StateMachine.add(
                states.DropOffItem.name,
                states.DropOffItem(**services),
                transitions={
                    outcomes.DROP_OFF_ITEM_SUCCESS: outcomes.CHALLENGE_SUCCESS,
                    outcomes.DROP_OFF_ITEM_FAILURE: outcomes.CHALLENGE_FAILURE
                },
                remapping={
                    'bin_id': 'current_bin',
                    'bin_data': 'bin_data',
                    'output_bin_data': 'bin_data',
                }
            )
        return sm

    def build_sm(self, **services):
        '''Builds the main state machine.

        You probably want to call either real_robot() or mock_robot() to build a
        state machine instead of this method.

        Args:
          tts: A text-to-speech publisher.
          tuck_arms: The tuck arms service proxy.
          move_torso: The torso service proxy.
          set_grippers: The grippers service proxy.
          move_head: The head service proxy.
        '''
        sm = smach.StateMachine(outcomes=[
            outcomes.CHALLENGE_SUCCESS,
            outcomes.CHALLENGE_FAILURE
        ])
        with sm:
            smach.StateMachine.add(
                states.StartPose.name,
                states.StartPose(**services),
                transitions={
                    outcomes.START_POSE_SUCCESS: states.FindShelf.name,
                    outcomes.START_POSE_FAILURE: outcomes.CHALLENGE_FAILURE
                },
                remapping={
                    'start_pose': 'start_pose'
                }
            )
            smach.StateMachine.add(
                states.FindShelf.name,
                states.FindShelf(**services),
                transitions={
                    outcomes.FIND_SHELF_SUCCESS: states.UpdatePlan.name,
                    outcomes.FIND_SHELF_FAILURE: outcomes.CHALLENGE_FAILURE
                },
                remapping={
                    'debug': 'debug',
                }
            )
            smach.StateMachine.add(
                states.UpdatePlan.name,
                states.UpdatePlan(**services),
                transitions={
                    outcomes.UPDATE_PLAN_NEXT_OBJECT: states.MoveToBin.name,
                    outcomes.UPDATE_PLAN_RELOCALIZE_SHELF: states.StartPose.name,
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
                states.MoveToBin(**services),
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
                states.SenseBin(**services),
                transitions={
                    outcomes.SENSE_BIN_SUCCESS: states.Grasp.name,
                    outcomes.SENSE_BIN_NO_OBJECTS: states.UpdatePlan.name,
                    outcomes.SENSE_BIN_FAILURE: states.UpdatePlan.name
                },
                remapping={
                    'bin_id': 'current_bin',
                    'clusters': 'clusters'
                }
            )
            smach.StateMachine.add(
                states.Grasp.name,
                states.Grasp(**services),
                transitions={
                    outcomes.GRASP_SUCCESS: states.ExtractItem.name,
                    outcomes.GRASP_FAILURE: (
                        states.UpdatePlan.name
                    )
                },
                remapping={
                    'bin_id': 'current_bin',
                    'clusters': 'clusters'
                }
            )
            smach.StateMachine.add(
                states.ExtractItem.name,
                states.ExtractItem(**services),
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
                states.DropOffItem(**services),
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
