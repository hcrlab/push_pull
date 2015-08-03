from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Vector3
from math import fabs
import rospy
from std_msgs.msg import Header
import tf
from RepositionAction import MoveArmStep, MoveBaseStep, RepositionAction


class PushSideways(RepositionAction):
    '''
    Stick the tool straight into the bin, then move it left or right.

    Possible applications:
    1. When target object is rotated <45 degrees from a graspable position,
    push sideways on the near end to rotate the object.
    '''
    # how far in front of item to position tool tip before attempting to touch object
    pre_application_dist = 0.05
    post_application_dist = 0.05

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

        # construct pre_application pose, application pose, and final pose
        ## be extra careful on edge bins
        self.frame = self.bounding_box.pose.header.frame_id
        pre_application_pose = Pose(
            position=Point(
                x=self.application_point.x + pre_application_delta.x - self.tool_length,
                y=self.cap_y(self.application_point.y + pre_application_delta.y),
                z=self.application_point.z + pre_application_delta.z,
            ),
            orientation=orientation,
        )
        application_pose = Pose(
            position=Point(
                x=self.application_point.x - self.tool_length,
                y=self.cap_y(self.application_point.y),
                z=self.application_point.z,
            ),
            orientation=orientation,
        )
        post_application_pose = Pose(
            position=Point(
                x=self.application_point.x + post_application_delta.x - self.tool_length,
                y=self.cap_y(self.centroid.y / 2.0),  # halfway between centroid and bin center
                z=self.application_point.z + post_application_delta.z,
            ),
            orientation=orientation,
        )

        push_application_pose = Pose(
            position=Point(
                x=self.application_point.x - self.tool_length,
                y=post_application_pose.position.y - application_pose.position.y,  # halfway between centroid and bin center
                z=self.application_point.z,
            ),
            orientation=orientation,
        )

        # figure out how far forward we have to move from current position
        # cover as much of that as we can by driving
        

        # for movement
        self.steps = [
            #MoveBaseStep(Vector3(x=drive_forward_distance, y=y_distance), self.frame),
            MoveArmStep(pre_application_pose, self.frame, False),
            MoveArmStep(application_pose, self.frame, False),
            MoveArmStep(push_application_pose, self.frame, False),
            #MoveBaseStep(Vector3(y=application_drive_sideways_distance), self.frame),
            #MoveArmStep(post_application_pose, self.frame, False),
            #MoveBaseStep(Vector3(y=-application_drive_sideways_distance), self.frame),
            MoveArmStep(pre_application_pose, self.frame, False),
            #MoveBaseStep(Vector3(x=-drive_forward_distance, y=-y_distance), self.frame),
        ]