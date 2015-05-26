from interactive_markers.interactive_marker_server import InteractiveMarkerServer
import mock
import rospy
from std_msgs.msg import String
import smach
import tf
from visualization_msgs.msg import Marker

from joint_states_listener.srv import ReturnJointStates
import outcomes
from pr2_pick_manipulation.srv import DriveAngular, DriveLinear, \
    DriveToPose, GetPose, MoveArm, MoveHead, MoveTorso, SetGrippers, \
    TuckArms, GetGrippers, MoveArmIk
from pr2_pick_perception.msg import Object
from pr2_pick_perception.srv import CropShelf, CropShelfResponse, \
    DeleteStaticTransform, FindCentroid, LocalizeShelf, LocalizeShelfResponse, \
    SetStaticTransform, PlanarPrincipalComponents, GetItemDescriptor, ClassifyTargetItem
from pr2_pick_perception.srv import CountPointsInBox
from pr2_pick_perception.srv import SegmentItems
from pr2_pick_contest.srv import GetItems, SetItems, GetTargetItems
from pr2_pick_contest.srv import LookupItem
from pr2_pretouch_optical_dist.srv import OpticalRefine
import states
from states.GraspTool import GraspTool, ReleaseTool


class StateMachineBuilder(object):

    # slugs to represent different state machines
    TEST_DROP_OFF_ITEM = 'test-drop-off-item'
    TEST_GRASP_TOOL = 'test-grasp-tool'
    TEST_MOVE_TO_BIN = 'test-move-to-bin'
    TEST_PUSH_ITEM = 'test-push-item'
    CAPTURE_ITEM_DESCRIPTOR = 'capture-item-descriptor'
    GATHER_DATA = 'gather-data'
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

        if self.state_machine_identifier == StateMachineBuilder.TEST_DROP_OFF_ITEM:
            build = self.build_sm_for_drop_off_item
        elif self.state_machine_identifier == StateMachineBuilder.TEST_GRASP_TOOL:
            build = self.build_sm_for_grasp_tool
        elif self.state_machine_identifier == StateMachineBuilder.TEST_MOVE_TO_BIN:
            build = self.build_sm_for_move_to_bin
        elif self.state_machine_identifier == StateMachineBuilder.TEST_PUSH_ITEM:
            build = self.build_sm_for_push_item
        elif self.state_machine_identifier == StateMachineBuilder.CAPTURE_ITEM_DESCRIPTOR:
            build = self.build_sm_for_item_descriptor_capture
        elif self.state_machine_identifier == StateMachineBuilder.GATHER_DATA:
            build = self.build_sm_for_gather_data
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
            'move_arm_ik': rospy.ServiceProxy('move_arm_ik', MoveArmIk),
            'set_grippers': rospy.ServiceProxy('set_grippers_service', SetGrippers),
            'get_grippers': rospy.ServiceProxy('get_grippers_service', GetGrippers),
            'tuck_arms': rospy.ServiceProxy('tuck_arms_service', TuckArms),
            'joint_states_listener': rospy.ServiceProxy('return_joint_states', ReturnJointStates),

            # World and Perception
            'crop_shelf': rospy.ServiceProxy('perception/shelf_cropper', CropShelf),
            'segment_items': rospy.ServiceProxy('perception/segment_items', SegmentItems),
            'find_centroid': rospy.ServiceProxy('perception/find_centroid', FindCentroid),
            'interactive_marker_server': InteractiveMarkerServer('pr2_pick_interactive_markers'),
            'localize_object': rospy.ServiceProxy('perception/localize_object',
                                                  LocalizeShelf),
            'markers': rospy.Publisher('pr2_pick_visualization', Marker),
            'set_static_tf': rospy.ServiceProxy('perception/set_static_transform',
                                                SetStaticTransform),
            'delete_static_tf': rospy.ServiceProxy('perception/delete_static_transform',
                                                 DeleteStaticTransform),
            'tf_listener': tf.TransformListener(),
            'get_planar_pca': rospy.ServiceProxy('planar_principal_components',
                                                 PlanarPrincipalComponents),
            'get_item_descriptor': rospy.ServiceProxy('perception/get_item_descriptor',
                                                 GetItemDescriptor),
            'classify_target_item': rospy.ServiceProxy('item_classifier/classify_target_item',
                                                       ClassifyTargetItem),
            'count_points_in_box': rospy.ServiceProxy('perception/count_points_in_box',
                                                       CountPointsInBox),
            'optical_detect_item': rospy.ServiceProxy('optical_detect_item', OpticalRefine),

            # Contest
            'get_items': rospy.ServiceProxy('inventory/get_items', GetItems),
            'set_items': rospy.ServiceProxy('inventory/set_items', SetItems),
            'get_target_items': rospy.ServiceProxy('inventory/get_target_items', GetTargetItems),
            'lookup_item': rospy.ServiceProxy('item_database/lookup_item', LookupItem)
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

        set_grippers = rospy.ServiceProxy('set_grippers_service', SetGrippers)
        set_grippers.wait_for_service = mock.Mock(return_value=None)
        set_grippers.call = mock.Mock(side_effect=self.side_effect('set_grippers'))

        move_head = rospy.ServiceProxy('move_head_service', MoveHead)
        move_head.wait_for_service = mock.Mock(return_value=None)
        move_head.call = mock.Mock(side_effect=self.side_effect('move_head'))

        moveit_move_arm = rospy.ServiceProxy('moveit_service', MoveArm)
        moveit_move_arm.wait_for_service = mock.Mock(return_value=None)
        moveit_move_arm.call = mock.Mock(
            side_effect=self.side_effect('moveit_move_arm'))

        move_arm_ik = rospy.ServiceProxy('move_arm_ik', MoveArm)
        move_arm_ik.wait_for_service = mock.Mock(return_value=None)
        move_arm_ik.call = mock.Mock(
            side_effect=self.side_effect('move_arm_ik'))

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
        interactive_marker.applyChanges = mock.Mock(
            side_effect=self.side_effect('server.applyChanges'))

        get_items = mock.Mock(side_effect=self.side_effect('get_items'))
        set_items = mock.Mock(side_effect=self.side_effect('set_items'))
        get_target_items = mock.Mock(side_effect=self.side_effect('get_target_items'))
        lookup_item = mock.Mock(side_effect=self.side_effect('lookup_item'))

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

            # Contest
            'get_items': get_items,
            'set_items': set_items,
            'get_target_items': get_target_items,
            'lookup_item': lookup_item
         }

    def mock_update_plan_execute(self, userdata):
        ''' Ask the user which bin to go to next. Don't worry about updating bin_data. '''
        default_next_bin = 'A'
        input_bin = raw_input('(Debug) Enter the next bin [{}]:'.format(default_next_bin))
        if len(input_bin) > 0 and input_bin[0] in 'ABCDEFGHIJKL':
            bin_letter = input_bin[0]
        else:
            bin_letter = default_next_bin

        userdata.next_bin = bin_letter
        return outcomes.UPDATE_PLAN_NEXT_OBJECT

    def build_interactive_bin_choosing_sm(self, success_transition, failure_transition, **services):
        '''
        Build state machine stub that starts by asking the user which bin to go to
        and going to that bin.
        @param success_transition - transition target for MoveToBin success
        @param failure_transition - transition target for MoveToBin failure
        '''
        sm = smach.StateMachine(outcomes=[
            outcomes.CHALLENGE_SUCCESS,
            outcomes.CHALLENGE_FAILURE
        ])

        mock_update_plan = states.UpdatePlan(**services)
        mock_update_plan.execute = self.mock_update_plan_execute

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
                    outcomes.MOVE_TO_BIN_SUCCESS: success_transition,
                    outcomes.MOVE_TO_BIN_FAILURE: failure_transition,
                },
                remapping={
                    'bin_id': 'current_bin'
                }
            )
        return sm

    def build_sm_for_move_to_bin(self, **services):
        ''' Minimal state machine for testing MoveToBin state '''
        return self.build_interactive_bin_choosing_sm(
            states.UpdatePlan.name,
            outcomes.CHALLENGE_FAILURE,
            **services
        )

    def build_sm_for_push_item(self, **services):
        sm = self.build_interactive_bin_choosing_sm(
            states.PushItem.name,
            outcomes.CHALLENGE_FAILURE,
            **services
        )
        with sm:
            smach.StateMachine.add(
                states.PushItem.name,
                states.PushItem(**services),
                transitions={
                    outcomes.PUSH_ITEM_SUCCESS: states.MoveToBin.name,
                    outcomes.PUSH_ITEM_FAILURE: states.MoveToBin.name,
                },
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

    def build_sm_for_grasp_tool(self, **services):
        ''' Test state machine for grasping tool '''
        sm = smach.StateMachine(outcomes=[
            outcomes.CHALLENGE_SUCCESS,
            outcomes.CHALLENGE_FAILURE
        ])
        with sm:
            smach.StateMachine.add(
                GraspTool.name,
                GraspTool(**services),
                transitions={
                    outcomes.GRASP_TOOL_SUCCESS: ReleaseTool.name,
                    outcomes.GRASP_TOOL_FAILURE: outcomes.CHALLENGE_FAILURE,
                },
            )
            smach.StateMachine.add(
                ReleaseTool.name,
                ReleaseTool(**services),
                transitions={
                    outcomes.RELEASE_TOOL_SUCCESS: outcomes.CHALLENGE_SUCCESS,
                    outcomes.RELEASE_TOOL_FAILURE: outcomes.CHALLENGE_FAILURE,
                },
            )
        return sm

    def build_sm_for_item_descriptor_capture(self, **services):
        '''State machine for calling CaptureItemDescriptor.
        '''
        sm = smach.StateMachine(outcomes=[
            outcomes.CHALLENGE_SUCCESS,
            outcomes.CHALLENGE_FAILURE
        ])

        mock_update_plan = states.UpdatePlan(**services)
        mock_update_plan.execute = self.mock_update_plan_execute

        with sm:
            smach.StateMachine.add(
                states.StartPose.name,
                states.StartPose(**services),
                transitions={
                    outcomes.START_POSE_SUCCESS: states.FindShelf.name,
                    outcomes.START_POSE_FAILURE: states.FindShelf.name
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
                    'bin_id': 'current_bin'
                }
            )
            smach.StateMachine.add(
                states.UpdatePlan.name,
                mock_update_plan,
                transitions={
                    outcomes.UPDATE_PLAN_NEXT_OBJECT: states.MoveToBin.name,
                    outcomes.UPDATE_PLAN_RELOCALIZE_SHELF: states.StartPose.name,
                    outcomes.UPDATE_PLAN_NO_MORE_OBJECTS: outcomes.CHALLENGE_SUCCESS,
                    outcomes.UPDATE_PLAN_FAILURE: outcomes.CHALLENGE_FAILURE
                },
                remapping={
                    'bin_data': 'bin_data',
                    'output_bin_data': 'bin_data',
                    'next_bin': 'current_bin',
                    'next_target' : 'current_target',
                    'next_bin_items': 'current_bin_items'
                }
            )
            smach.StateMachine.add(
                states.MoveToBin.name,
                states.MoveToBin(**services),
                transitions={
                    outcomes.MOVE_TO_BIN_SUCCESS: states.CaptureItemDescriptor.name,
                    outcomes.MOVE_TO_BIN_FAILURE: outcomes.CHALLENGE_FAILURE
                },
                remapping={
                    'bin_id': 'current_bin'
                }
            )
            smach.StateMachine.add(
                states.CaptureItemDescriptor.name,
                states.CaptureItemDescriptor(**services),
                transitions={
                    outcomes.CAPTURE_ITEM_NEXT: states.CaptureItemDescriptor.name,
                    outcomes.CAPTURE_ITEM_DONE: states.StartPose.name
                },
                remapping={
                    'bin_id': 'current_bin',
                    'clusters': 'clusters'
                }
            )

        return sm

    def build_sm_for_gather_data(self, **services):
        '''State machine for calling GatherData.
        '''
        sm = smach.StateMachine(outcomes=[
            outcomes.CHALLENGE_SUCCESS,
            outcomes.CHALLENGE_FAILURE
        ])

        mock_update_plan = states.UpdatePlan(**services)
        mock_update_plan.execute = self.mock_update_plan_execute

        with sm:
            smach.StateMachine.add(
                states.StartPose.name,
                states.StartPose(**services),
                transitions={
                    outcomes.START_POSE_SUCCESS: states.FindShelf.name,
                    outcomes.START_POSE_FAILURE: states.FindShelf.name
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
                    'bin_id': 'current_bin'
                }
            )
            smach.StateMachine.add(
                states.UpdatePlan.name,
                mock_update_plan,
                transitions={
                    outcomes.UPDATE_PLAN_NEXT_OBJECT: states.MoveToBin.name,
                    outcomes.UPDATE_PLAN_RELOCALIZE_SHELF: states.StartPose.name,
                    outcomes.UPDATE_PLAN_NO_MORE_OBJECTS: outcomes.CHALLENGE_SUCCESS,
                    outcomes.UPDATE_PLAN_FAILURE: outcomes.CHALLENGE_FAILURE
                },
                remapping={
                    'bin_data': 'bin_data',
                    'output_bin_data': 'bin_data',
                    'next_bin': 'current_bin',
                    'next_target' : 'current_target',
                    'next_bin_items': 'current_bin_items'
                }
            )
            smach.StateMachine.add(
                states.MoveToBin.name,
                states.MoveToBin(**services),
                transitions={
                    outcomes.MOVE_TO_BIN_SUCCESS: states.GatherData.name,
                    outcomes.MOVE_TO_BIN_FAILURE: outcomes.CHALLENGE_FAILURE
                },
                remapping={
                    'bin_id': 'current_bin'
                }
            )
            smach.StateMachine.add(
                states.GatherData.name,
                states.GatherData(**services),
                transitions={
                    outcomes.GATHER_DATA_NEXT: states.GatherData.name,
                    outcomes.GATHER_DATA_DONE: states.StartPose.name
                },
                remapping={
                    'bin_id': 'current_bin',
                    'current_target': 'current_target',
                    'current_bin_items': 'current_bin_items',
                    'clusters': 'clusters'
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
                    outcomes.START_POSE_FAILURE: states.StartPose.name
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
                    outcomes.FIND_SHELF_FAILURE: states.StartPose.name
                },
                remapping={
                    'debug': 'debug',
                    'bin_id': 'current_bin'
                }
            )
            smach.StateMachine.add(
                states.UpdatePlan.name,
                states.UpdatePlan(**services),
                transitions={
                    outcomes.UPDATE_PLAN_NEXT_OBJECT: states.MoveToBin.name,
                    outcomes.UPDATE_PLAN_RELOCALIZE_SHELF: states.StartPose.name,
                    outcomes.UPDATE_PLAN_NO_MORE_OBJECTS: outcomes.CHALLENGE_SUCCESS,
                    outcomes.UPDATE_PLAN_FAILURE: states.StartPose.name
                },
                remapping={
                    'bin_data': 'bin_data',
                    'output_bin_data': 'bin_data',
                    'next_bin': 'current_bin',
                    'next_target' : 'current_target',
                    'next_bin_items': 'current_bin_items'
                }
            )
            smach.StateMachine.add(
                states.MoveToBin.name,
                states.MoveToBin(**services),
                transitions={
                    outcomes.MOVE_TO_BIN_SUCCESS: states.SenseBin.name,
                    outcomes.MOVE_TO_BIN_FAILURE: states.UpdatePlan.name
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
                    'current_target': 'current_target',
                    'current_bin_items': 'current_bin_items',
                    'clusters': 'clusters',
                    'target_cluster': 'target_cluster',
                    'target_descriptor': 'target_descriptor',
                    'target_model': 'target_model'
                }
            )
            smach.StateMachine.add(
                states.Grasp.name,
                states.Grasp(**services),
                transitions={
                    outcomes.GRASP_SUCCESS: states.ExtractItem.name,
                    outcomes.GRASP_NONE: states.SenseBin.name,
                    outcomes.GRASP_FAILURE: (
                        states.UpdatePlan.name
                    )
                },
                remapping={
                    'bin_id': 'current_bin',
                    'target_cluster': 'target_cluster',
                    'current_target': 'current_target',
                    'item_model': 'target_model',
                    'target_descriptor': 'target_descriptor'
                }
            )
            smach.StateMachine.add(
                states.ExtractItem.name,
                states.ExtractItem(**services),
                transitions={
                    outcomes.EXTRACT_ITEM_SUCCESS: states.VerifyGrasp.name,
                    outcomes.EXTRACT_ITEM_FAILURE: states.UpdatePlan.name
                },
                remapping={
                    'bin_id': 'current_bin'
                }
            )
            smach.StateMachine.add(
                states.VerifyGrasp.name,
                states.VerifyGrasp(**services),
                transitions={
                    outcomes.VERIFY_GRASP_SUCCESS: states.DropOffItem.name,
                    outcomes.VERIFY_GRASP_FAILURE: states.UpdatePlan.name,
                    outcomes.VERIFY_GRASP_RETRY: states.MoveToBin.name,
                },
                remapping={
                    'bin_id': 'current_bin',
                    'bin_data': 'bin_data',
                    'output_bin_data': 'bin_data',
                    'current_target': 'current_target',
                    'item_model': 'target_model'
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
                    'output_bin_data': 'bin_data'
                }
            )
        return sm
