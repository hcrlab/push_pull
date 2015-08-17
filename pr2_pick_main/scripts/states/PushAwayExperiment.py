from copy import deepcopy
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from math import fabs
from moveit_msgs.srv import GetPositionIKRequest
import rospy

from RepositionAction import MoveArmStep, MoveBaseStep, RepositionAction

class PushAwayExperiment(RepositionAction):
    '''
    Pushes toward the rear of the shelf, starting at the given application point.

    Possible applications:
    1. When target object is rotated >45 degrees from a graspable position,
    push on a point between centroid and the far end, to rotate the object.
    2. Push a non target object to the back of the bin to create space around
    target object so we can grasp it.
    '''

    bin_depth = 0.42
    dist_tolerance = 0.02
    # how far in front of item to position tool tip before attempting to touch object
    pre_application_dist = 0.05

    #### ACTION PARAMETERS
    pushing_distance = 0.08

    def set_params(self, distance_to_push):
        self.pushing_distance = distance_to_push

    def build_trajectory(self):
        '''
        Construct waypoints for wrist_roll_link from pre/application/post points
        '''

        # where to be before moving to application point, relative to application point
        pre_application_delta = Vector3(0, 0, 0)
        # where to move after application point, relative to application point
        post_application_delta = Vector3(0, 0, 0)
        # orientation of gripper when repositioning, relative to current orientation
        orientation = Quaternion(1.0, 0.0, 0.0, 0.0)

        pre_application_delta.x = -self.pre_application_dist

        post_application_delta.x = min(
            (3.0 / 4.0) * self.bounding_box.dimensions.x,
            self.bin_depth - self.application_point.x - self.bounding_box.dimensions.y
        )

        # construct pre_application pose, application pose, and final pose
        ## TODO: be extra careful on edge bins
        self.frame = self.bounding_box.pose.header.frame_id
        pre_application_pose = Pose(
            position=Point(
                x=self.application_point.x + pre_application_delta.x - self.tool_length - self.pushing_distance/2,
                y=self.cap_y(self.application_point.y + pre_application_delta.y),
                z=self.application_point.z + pre_application_delta.z,
            ),
            orientation=orientation,
        )
        application_pose = Pose(
            position=Point(
                x=self.application_point.x - self.tool_length + self.pushing_distance/2,
                y=self.cap_y(self.application_point.y),
                z=self.application_point.z,
            ),
            orientation=orientation,
        )
        post_application_pose = Pose(
            position=Point(
                x=self.application_point.x + post_application_delta.x - self.tool_length,
                y=self.cap_y(self.application_point.y + post_application_delta.y),
                z=self.application_point.z + post_application_delta.z,
            ),
            orientation=orientation,
        )

        # for visualization
        self.trajectory = [
            pre_application_pose, application_pose, post_application_pose]

        # for actually moving
        self.steps = [
            MoveArmStep(pre_application_pose, self.frame, False),
            MoveArmStep(application_pose, self.frame, False),
            MoveArmStep(pre_application_pose, self.frame, False),
        ]
