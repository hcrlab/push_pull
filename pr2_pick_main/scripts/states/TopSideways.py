from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from math import fabs
import tf
import math
from RepositionAction import RepositionAction
from RepositionAction import MoveArmStep, MoveBaseStep, RepositionAction

class TopSideways(RepositionAction):
    '''
    Parent class for actions that push away on some point of the object.
    Child classes are parameterized by where on the object they're pushing.
    '''
    bin_depth = 0.42
    dist_tolerance = 0.02
    # distance from tip of tool to wrist roll link
    tool_length = 0.42
    # how far in front of item to position tool tip before attempting to touch object
    pre_application_dist = 0.05
    # dist to push down on object when pulling forward
    push_down_offset = 0.02
    # how close to the edge of the shelf to pull the tip of the object
    distance_from_edge = 0.05

    def build_trajectory(self):
        '''
        Construct waypoints for wrist_roll_link from pre/application/post points
        '''
        # where to be before moving to application point, relative to application point
        pre_application_delta = Vector3(0, 0, 0)
        # where to move after application point, relative to application point
        post_application_delta = Vector3(0, 0, 0)
        # orientation of gripper when repositioning, relative to current orientation

        quaternion = tf.transformations.quaternion_from_euler(math.pi / 2 , 0.0, 0.0)
        #type(pose) = geometry_msgs.msg.Pose
        orientation = Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])

        pre_application_delta.x = -self.pre_application_dist
        pre_application_delta.z = self.push_down_offset 

        post_application_delta.x = - self.centroid.x + (self.bounding_box.dimensions.x / 2.0) \
            + self.distance_from_edge


        # construct pre_application pose, application pose, and final pose
        ## TODO: correct poses to prevent hitting the bin wall
        ## be extra careful on edge bins
        self.frame = self.bounding_box.pose.header.frame_id
        pre_application_pose = Pose(
            position=Point(
                x=self.application_point.x - 0.015 - self.tool_length,
                y=self.application_point.y,
                z=self.application_point.z + pre_application_delta.z,
            ),
            orientation=orientation,
        )
        above_application_pose = Pose(
            position=Point(
                x=self.application_point.x - self.tool_length - 0.01,
                y=self.application_point.y,
                z=self.application_point.z,
            ),
            orientation=orientation,
        )
        application_pose = Pose(
            position=Point(
                x=self.application_point.x - self.tool_length - 0.01,
                y=self.application_point.y,
                z=self.application_point.z - 0.005,
            ),
            orientation=orientation,
        )
        pre_pull_pose = Pose(
            position=Point(
                x=self.application_point.x - self.tool_length,
                y=self.application_point.y - 0.01,
                z=self.application_point.z - 0.005,
            ),
            orientation=orientation,
        )
        pull_pose = Pose(
            position=Point(
                x=self.application_point.x - self.tool_length,
                y=self.application_point.y - 0.025,
                z=self.application_point.z - 0.005,
            ),
            orientation=orientation,
        )
        lift_pose = Pose(
            position=Point(
                x=self.application_point.x - self.tool_length,
                y=self.application_point.y - 0.025,
                z=self.application_point.z + pre_application_delta.z,
            ),
            orientation=orientation,
        )
        self.trajectory = [
            pre_application_pose, above_application_pose, application_pose, pull_pose, lift_pose, pre_application_pose]
        # Probably want to change this experimentally
        self.collision_checking = [True, True, True, True, True, True]

        self.steps = [
            MoveArmStep(pre_application_pose, self.frame, False),
            MoveArmStep(above_application_pose, self.frame, False),
            MoveArmStep(application_pose, self.frame, False),
            MoveArmStep(pre_pull_pose, self.frame, False),
            MoveArmStep(pull_pose, self.frame, False),
            MoveArmStep(lift_pose, self.frame, False)
        ]