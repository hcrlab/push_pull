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
        side = raw_input("1. Left \n2. Right \n")
        if(side == '1'):
            back_end = self.ends[3]
            front_end = self.ends[1]
            if(self.ends[3].y < self.ends[1].y):
                rospy.loginfo("3 < 1")
                target_end = self.ends[3]
            else:
                target_end = self.ends[1]    

            push_direction_sign = 1
        else:
            back_end = self.ends[2]
            front_end = self.ends[0]
            if(self.ends[2].y < self.ends[0].y):
                rospy.loginf("2 < 0")
                target_end = self.ends[2]
            else:
                target_end = self.ends[0]  
            push_direction_sign = -1

        self.application_height = 0.09  

        # where to be before moving to application point, relative to application point
        pre_application_delta = Vector3(0, 0, 0)
        # where to move after application point, relative to application point
        post_application_delta = Vector3(0, 0, 0)
        # orientation of gripper when repositioning, relative to current orientation
        orientation = Quaternion(1.0, 0.0, 0.0, 0.0)

        pre_application_delta.y = 0.02

        distance_from_end = float(raw_input("Distance to push the object: "))

        distance_x = back_end.x - self.tool_length

        if(self.action_name == 'push_point_contact'):
            distance_x =  front_end.x + float(raw_input("Distance from front end of object to apply tool: ")) \
                        - self.tool_length

        # construct pre_application pose, application pose, and final pose
        ## be extra careful on edge bins
        self.frame = self.bounding_box.pose.header.frame_id
        pre_application_pose = Pose(
            position=Point(
                x=distance_x - 0.10,
                y=self.cap_y(target_end.y  + (pre_application_delta.y * push_direction_sign)),
                z=self.application_height
            ),
            orientation=orientation,
        )

        application_pose = Pose(
            position=Point(
                x=distance_x,
                y=self.cap_y(target_end.y  + (pre_application_delta.y * push_direction_sign)),
                z=self.application_height
            ),
            orientation=orientation,
        )

        push_application_pose = Pose(
            position=Point(
                x=distance_x,
                y=target_end.y - (distance_from_end * push_direction_sign),  # halfway between centroid and bin center
                z=self.application_height,
            ),
            orientation=orientation,
        )

        # for movement
        self.steps = [
            MoveArmStep(pre_application_pose, self.frame, False),
            MoveArmStep(application_pose, self.frame, False),
            MoveArmStep(push_application_pose, self.frame, False),
            MoveArmStep(pre_application_pose, self.frame, False)
        ]