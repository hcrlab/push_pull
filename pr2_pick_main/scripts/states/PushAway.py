from copy import deepcopy
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from math import fabs
from moveit_msgs.srv import GetPositionIKRequest
import rospy

from RepositionAction import MoveArmStep, MoveBaseStep, RepositionAction

class PushAway(RepositionAction):
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

    def build_trajectory(self):
        '''
        Construct waypoints for wrist_roll_link from pre/application/post points
        '''

        self.application_point = self.application_point.point

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
                x=self.application_point.x + pre_application_delta.x - self.tool_length - 0.04,
                y=self.cap_y(self.application_point.y + pre_application_delta.y),
                z=self.application_point.z + pre_application_delta.z,
            ),
            orientation=orientation,
        )
        application_pose = Pose(
            position=Point(
                x=self.application_point.x - self.tool_length + 0.04,
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
            #MoveArmStep(post_application_pose, self.frame, False),
            MoveArmStep(pre_application_pose, self.frame, False),
        ]

        def get_closest_accessible_pose_x(self, desired_pose, frame, max_perturbation):
            '''
            Decrease the x value until we get something reachable. Don't change
            the pose by more than max_perturbation. Assumes max_perturbation
            corresponds with a known reachable pose.
            '''
            ik_client = self.services['ik_client']
            perturbation_increment = 0.02

            experimental_pose = deepcopy(desired_pose)

            for perturbation in range(0, max_perturbation, perturbation_increment):
                experimental_pose = deepcopy(desired_pose)
                experimental_pose.position.x -= perturbation
                experimental_pose_stamped = PoseStamped(
                    header=Header(frame_id=frame),
                    pose=experimental_pose,
                )
                ik_request = GetPositionIKRequest()
                ik_request.ik_request.group_name = 'left_arm'
                ik_request.ik_request.timeout = rospy.Duration(3.0)
                ik_request.ik_request.pose_stamped = experimental_pose_stamped
                ik_response = ik_client(ik_request)

                if ik_response.error_code.val == ik_response.error_code.SUCCESS:
                    return perturbation

            return max_perturbation 
