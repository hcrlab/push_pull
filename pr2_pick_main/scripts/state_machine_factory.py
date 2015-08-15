from interactive_markers.interactive_marker_server import InteractiveMarkerServer
import rospy
from std_msgs.msg import String
import smach
import tf
from visualization_msgs.msg import Marker
from moveit_msgs.msg import AttachedCollisionObject, PlanningScene
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
import states
from states.GraspTool import GraspTool, ReleaseTool
#from states.Simulation import Simulation
from states.MoveObject import MoveObject
from convert_pcl.srv import ConvertPCL
from moveit_msgs.srv import GetPlanningScene, GetPositionFK, GetPositionIK

class StateMachineBuilder(object):

    # slugs to represent different state machines
    TEST_GRASP_TOOL = 'test-grasp-tool'
    PLAN_GRASP = 'plan-grasp'
    DEFAULT = 'default'
    SIMULATION = 'simulation'
    EXPLORE = 'explore'

    def __init__(self):
        self.state_machine_identifier = StateMachineBuilder.DEFAULT

    def set_state_machine(self, state_machine_identifier):
        ''' Sets which state machine to use. '''
        self.state_machine_identifier = state_machine_identifier
        return self

    def build(self):
        ''' Build the state machine with previously specified mock status and
        state machine type. '''
        services = self.real_robot_services()

        if self.state_machine_identifier == StateMachineBuilder.TEST_GRASP_TOOL:
            build = self.build_sm_for_grasp_tool
        elif self.state_machine_identifier == StateMachineBuilder.PLAN_GRASP:
            build = self.build_sm_grasp_planner
        elif self.state_machine_identifier == StateMachineBuilder.SIMULATION:
            build = self.build_sm_for_simulation
        elif self.state_machine_identifier == StateMachineBuilder.EXPLORE:
            build = self.build_sm_for_explore
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
            'attached_collision_objects': rospy.Publisher('/attached_collision_object', AttachedCollisionObject),
            'ik_client': rospy.ServiceProxy('compute_ik', GetPositionIK),


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
	    'get_planning_scene': rospy.ServiceProxy('/get_planning_scene', GetPlanningScene),
            # Contest
            #'get_items': rospy.ServiceProxy('inventory/get_items', GetItems),
            #'set_items': rospy.ServiceProxy('inventory/set_items', SetItems),
            'get_target_items': rospy.ServiceProxy('inventory/get_target_items', GetTargetItems),
            'lookup_item': rospy.ServiceProxy('item_database/lookup_item', LookupItem),
	    'convert_pcl_service': rospy.ServiceProxy('convert_pcl_service', ConvertPCL),
            'planning_scene_publisher': rospy.Publisher('planning_scene', PlanningScene)
	}

    def build_sm_for_grasp_tool(self, **services):
        ''' Test state machine for grasping tool '''
        rospy.loginfo("\n in state machine! \n")
        sm = smach.StateMachine(outcomes=[
            outcomes.CHALLENGE_SUCCESS,
            outcomes.CHALLENGE_FAILURE
        ])
        with sm:
            smach.StateMachine.add(
                GraspTool.name,
                GraspTool(**services),
                transitions={
                    outcomes.GRASP_TOOL_SUCCESS: outcomes.CHALLENGE_SUCCESS,
                    outcomes.GRASP_TOOL_FAILURE: outcomes.CHALLENGE_FAILURE,
                }
            )

        return sm

    def build_sm_for_simulation(self, **services):
            ''' Test state machine for simulation '''

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
                        outcomes.FIND_SHELF_FAILURE: outcomes.CHALLENGE_FAILURE
                    },
                    remapping={
                        'debug': 'debug',
                        'bin_id': 'current_bin'
                    }
                )
                smach.StateMachine.add(
                    states.UpdatePlanExperiment.name,
                    states.UpdatePlanExperiment(**services),
                    transitions={
                        outcomes.UPDATE_PLAN_NEXT_OBJECT: states.SenseBin.name,
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
                    states.SenseBin.name,
                    states.SenseBin(**services),
                    transitions={
                        outcomes.SENSE_BIN_SUCCESS: states.GraspPlanner.name,
                        outcomes.SENSE_BIN_NO_OBJECTS: outcomes.CHALLENGE_FAILURE,
                        outcomes.SENSE_BIN_FAILURE: outcomes.CHALLENGE_FAILURE
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
                    states.GraspPlanner.name,
                    states.GraspPlanner(**services),
                    transitions={
                        outcomes.GRASP_PLAN_SUCCESS: states.ExtractItem.name,
                        outcomes.GRASP_PLAN_NONE: states.SenseBin.name,
                        outcomes.GRASP_PLAN_FAILURE: states.SenseBin.name,
                        outcomes.GRASP_MOVE_OBJECT: states.MoveObject.name 
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
                    MoveObject.name,
                    MoveObject(**services),
                    transitions={
                        outcomes.MOVE_OBJECT_SUCCESS: states.SenseBin.name,
                        outcomes.MOVE_OBJECT_FAILURE: states.SenseBin.name,
                    }
                )
           
	        smach.StateMachine.add(
                states.ExtractItem.name,
                states.ExtractItem(**services),
                transitions={
                    outcomes.EXTRACT_ITEM_SUCCESS: states.DropOffItem.name,
                    outcomes.EXTRACT_ITEM_FAILURE: states.SenseBin.name
                },
                remapping={
                    'bin_id': 'current_bin',
                    'item_model': 'target_model'
                }
                )
                smach.StateMachine.add(
                states.DropOffItem.name,
                states.DropOffItem(**services),
                transitions={
                    outcomes.DROP_OFF_ITEM_SUCCESS: states.StartPose.name,
                    outcomes.DROP_OFF_ITEM_FAILURE: states.StartPose.name
                },
                remapping={
                    'bin_id': 'current_bin',
                    'bin_data': 'bin_data',
                    'output_bin_data': 'bin_data'
                }
                )
            return sm


    def build_sm_for_explore(self, **services):
            ''' Test state machine for simulation '''

            sm = smach.StateMachine(outcomes=[
                outcomes.CHALLENGE_SUCCESS,
                outcomes.CHALLENGE_FAILURE
            ])

            with sm:
            	smach.StateMachine.add(
                	states.StartPoseExplore.name,
                	states.StartPoseExplore(**services),
                	transitions={
                    	outcomes.START_POSE_SUCCESS: states.FindShelf.name,
                    	outcomes.START_POSE_FAILURE: states.StartPoseExplore.name
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
                    states.UpdatePlan(**services),
                    transitions={
                        outcomes.UPDATE_PLAN_NEXT_OBJECT: states.SenseObjectBefore.name,
                        outcomes.UPDATE_PLAN_FAILURE: states.StartPoseExplore.name
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
                    states.SenseObjectBefore.name,
                    states.SenseObjectBefore(**services),
                    transitions={
                        outcomes.SENSE_BIN_SUCCESS: states.ExploreToolActions.name,
                        outcomes.SENSE_BIN_NO_OBJECTS: states.StartPoseExplore,
                        outcomes.SENSE_BIN_FAILURE: states.StartPoseExplore,
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
                # smach.StateMachine.add(
                #     states.PrepareToolAction.name,
                #     states.PrepareToolAction(**services),
                #     transitions={
                #         outcomes.PREPARE_TOOL_ACTION_SUCCESS: states.ExploreToolActions.name 
                #         outcomes.PREPARE_TOOL_ACTION_FAILURE: states.SenseObjectBefore.name,
                #     },
                #     remapping={
                #         'bin_id': 'current_bin',
                #         'target_cluster': 'target_cluster',
                #         'current_target': 'current_target',
                #         'item_model': 'target_model',
                #         'target_descriptor': 'target_descriptor'
                #     }
                # )
                smach.StateMachine.add(
                    ExploreToolActions.name,
                    ExploreToolActions(**services),
                    transitions={
                        outcomes.TOOL_ACTION_SUCCESS: states.SenseObjectAfter.name,
                        outcomes.TOOL_ACTION_FAILURE: states.SenseObjectBefore.name,
                    }
                )
                smach.StateMachine.add(
                    SenseObjectAfter.name,
                    SenseObjectAfter(**services),
                    transitions={
                        outcomes.TOOL_ACTION_SUCCESS: states.SenseObjectBefore.name,
                        outcomes.TOOL_ACTION_FAILURE: states.SenseObjectBefore.name,
                    }
                )
           
            return sm
