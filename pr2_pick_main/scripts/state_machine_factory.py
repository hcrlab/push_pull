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
from states.MoveObjectExperiment import MoveObjectExperiment
from convert_pcl.srv import ConvertPCL
from moveit_msgs.srv import GetPlanningScene, GetPositionFK, GetPositionIK

class StateMachineBuilder(object):

    # slugs to represent different state machines
    EXPLORE = 'explore'
    EXPERIMENT = 'experiment'
    PLAN = 'plan'

    def __init__(self):
        self.state_machine_identifier = StateMachineBuilder.EXPLORE

    def set_state_machine(self, state_machine_identifier):
        ''' Sets which state machine to use. '''
        self.state_machine_identifier = state_machine_identifier
        return self

    def build(self):
        ''' Build the state machine with previously specified mock status and
        state machine type. '''
        services = self.real_robot_services()

        if self.state_machine_identifier == StateMachineBuilder.EXPLORE:
            build = self.build_sm_for_explore
        elif self.state_machine_identifier == StateMachineBuilder.PLAN:
            build = self.build_sm_for_plan
        elif self.state_machine_identifier == StateMachineBuilder.EXPERIMENT:
            build = self.build_sm_for_experiment
        else:
            build = self.build_sm_for_simulation


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

    def build_sm_for_explore(self, **services):
            ''' Test state machine for exploring the different parameters '''

            sm = smach.StateMachine(outcomes=[
                outcomes.EXPLORATION_SUCCESS,
                outcomes.EXPLORATION_FAILURE
            ])

            with sm:
            	smach.StateMachine.add(
                	states.InitializeExploration.name,
                	states.InitializeExploration(**services),
                	transitions={
                    	outcomes.INITIALIZE_SUCCESS: states.SenseObject.name,
                    	outcomes.INITIALIZE_FAILURE: outcomes.EXPLORATION_FAILURE
                	}
            	)
                smach.StateMachine.add(
                    states.SenseObject.name,
                    states.SenseObject(**services),
                    transitions={
                        outcomes.SENSE_OBJECT_BEFORE_SUCCESS: states.ExploreToolActions.name,
                        outcomes.SENSE_OBJECT_AFTER_SUCCESS: states.SenseObject.name,
                        outcomes.SENSE_OBJECT_FAILURE: states.InitializeExploration.name,
                    }
                )
                smach.StateMachine.add(
                    states.ExploreToolActions.name,
                    states.ExploreToolActions(**services),
                    transitions={
                        outcomes.TOOL_EXPLORATION_SUCCESS: states.SenseObject.name,
                        outcomes.TOOL_EXPLORATION_FAILURE: states.SenseObject.name,
                    }
                )
           
            return sm
    def build_sm_for_plan(self, **services):
            ''' Test state machine for exploring the different parameters '''

            sm = smach.StateMachine(outcomes=[
                outcomes.PLAN_SUCCESS,
                outcomes.PLAN_FAILURE
            ])

            with sm:
                smach.StateMachine.add(
                    states.InitializeExploration.name,
                    states.InitializeExploration(**services),
                    transitions={
                        outcomes.INITIALIZE_SUCCESS: states.SenseObject.name,
                        outcomes.INITIALIZE_FAILURE: outcomes.PLAN_FAILURE
                    }
                )
                smach.StateMachine.add(
                    states.SenseObject.name,
                    states.SenseObject(**services),
                    transitions={
                        outcomes.SENSE_OBJECT_BEFORE_SUCCESS: states.CreatePlan.name,
                        outcomes.SENSE_OBJECT_AFTER_SUCCESS: states.CreatePlan.name,
                        outcomes.SENSE_OBJECT_FAILURE: states.InitializeExploration.name,
                    }
                )
                smach.StateMachine.add(
                    states.CreatePlan.name,
                    states.CreatePlan(**services),
                    transitions={
                        outcomes.CREATE_PLAN_TOOL: states.ExploreToolActions.name,
                        outcomes.CREATE_PLAN_GRASP: states.ExecuteGrasp.name,
                    }
                )
                smach.StateMachine.add(
                    states.ExecuteGrasp.name,
                    states.ExecuteGrasp(**services),
                    transitions={
                        outcomes.EXECUTE_GRASP_SUCCESS: states.SenseObject.name,
                        outcomes.EXECUTE_GRASP_FAILURE: states.InitializeExploration.name,
                    }
                )
                smach.StateMachine.add(
                    states.ExploreToolActions.name,
                    states.ExploreToolActions(**services),
                    transitions={
                        outcomes.TOOL_EXPLORATION_SUCCESS: states.SenseObject.name,
                        outcomes.TOOL_EXPLORATION_FAILURE: states.SenseObject.name,
                    }
                )
           
            return sm


    def build_sm_for_experiment(self, **services):
            ''' Test state machine for collecting data '''

            sm = smach.StateMachine(outcomes=[
                outcomes.EXPERIMENT_SUCCESS,
                outcomes.EXPERIMENT_FAILURE
            ])

            with sm:
                smach.StateMachine.add(
                    states.InitializeExploration.name,
                    states.InitializeExploration(**services),
                    transitions={
                        outcomes.INITIALIZE_SUCCESS: states.UpdatePlan.name,
                        outcomes.INITIALIZE_FAILURE: outcomes.EXPERIMENT_FAILURE
                    }
                )
                smach.StateMachine.add(
                    states.UpdatePlan.name,
                    states.UpdatePlan(**services),
                    transitions={
                        outcomes.UPDATE_PLAN_NEXT_OBJECT: states.SenseObject.name,
                        outcomes.UPDATE_PLAN_FAILURE: outcomes.EXPERIMENT_FAILURE
                    }
                )
                smach.StateMachine.add(
                    states.SenseObject.name,
                    states.SenseObject(**services),
                    transitions={
                        outcomes.SENSE_OBJECT_BEFORE_SUCCESS: states.ExploreToolActions.name,
                        outcomes.SENSE_OBJECT_AFTER_SUCCESS: states.UpdatePlan.name,
                        outcomes.SENSE_OBJECT_FAILURE: states.InitializeExploration.name,
                    }
                )
                smach.StateMachine.add(
                    states.ExploreToolActions.name,
                    states.ExploreToolActions(**services),
                    transitions={
                        outcomes.TOOL_EXPLORATION_SUCCESS: states.SenseObject.name,
                        outcomes.TOOL_EXPLORATION_FAILURE: states.SenseObject.name,
                    }
                )
           
            return sm


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
                    states.StartPoseExperiment.name,
                    states.StartPoseExperiment(**services),
                    transitions={
                        outcomes.START_POSE_SUCCESS: states.FindShelf.name,
                        outcomes.START_POSE_FAILURE: states.StartPoseExperiment.name
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
                        outcomes.UPDATE_PLAN_NEXT_OBJECT: states.SenseBinExperiment.name,
                        outcomes.UPDATE_PLAN_FAILURE: states.StartPoseExperiment.name
                    },
                    remapping={
                        'bin_data': 'bin_data',
                        'output_bin_data': 'bin_data',
                        'next_bin': 'current_bin',
                        'next_target' : 'current_target',
                        'next_bin_items': 'current_bin_items',
                        'current_trial' : 'current_trial'
                    }
                )
                smach.StateMachine.add(
                    states.SenseBinExperiment.name,
                    states.SenseBinExperiment(**services),
                    transitions={
                        outcomes.SENSE_BIN_SUCCESS: states.GraspPlannerExperiment.name,
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
                    states.GraspPlannerExperiment.name,
                    states.GraspPlannerExperiment(**services),
                    transitions={
                        outcomes.GRASP_PLAN_NONE: states.SenseBinExperiment.name,
                        outcomes.GRASP_PLAN_FAILURE: states.SenseBinExperiment.name,
                        outcomes.GRASP_MOVE_OBJECT: states.MoveObjectExperiment.name 
                    },
                    remapping={
                        'bin_id': 'current_bin',
                        'target_cluster': 'target_cluster',
                        'current_target': 'current_target',
                        'item_model': 'target_model',
                        'target_descriptor': 'target_descriptor',
                        'before_record' : 'before_record'
                    }
                )
                smach.StateMachine.add(
                    MoveObjectExperiment.name,
                    MoveObjectExperiment(**services),
                    transitions={
                        outcomes.MOVE_OBJECT_SUCCESS: states.UpdatePlanExperiment.name,
                        outcomes.MOVE_OBJECT_FAILURE: states.UpdatePlanExperiment.name,
                    },
                    remapping={
                        'bin_id': 'current_bin',
                        'target_cluster': 'target_cluster',
                        'current_target': 'current_target',
                        'item_model': 'target_model',
                        'target_descriptor': 'target_descriptor',
                        'current_trial' : 'current_trial',
                        'before_record' : 'before_record',
                        'current_trial_num' : 'current_trial_num'
                    }
                )

            return sm

