import outcomes
import rospy
import smach
from pr2_pick_main import handle_service_exceptions
import rosbag
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from actionlib import SimpleActionClient
import tf
import math
from geometry_msgs.msg import Point,PointStamped, Pose, PoseStamped
import visualization as viz
from visualization import IdTable
from std_msgs.msg import Header
from PushAway import PushAway
from PullForward import PullForward
from PushSideways import PushSideways
from TopSideways import TopSideways

class Simulation(smach.State):
    """Sets the robot's starting pose at the beginning of the challenge.
    """
    name = 'SIMULATION'

    def __init__(self, tts, tuck_arms, move_head, tf_listener, **services):
        """Constructor for this state.

        Args:
          tts: A publisher for the TTS node
          tuck_arms: The tuck arms service proxy.
          move_head: The head service proxy.
        """
        smach.State.__init__(
            self,
            outcomes=[outcomes.SIMULATION_SUCCESS, outcomes.SIMULATION_FAILURE],
            input_keys=['debug'])

        self._markers = services['markers']
        for key in services.keys():
            rospy.loginfo(key)
        self._tf_listener = tf.TransformListener()
        self.services = services

    def _publish(self, marker):
        """Publishes a marker to the given publisher.

        We need to wait for rviz to subscribe. If there are no subscribers to the
        topic within 5 seconds, we give up.
        """

        rate = rospy.Rate(1)
        for i in range(5):
            if self._markers.get_num_connections() > 0:
                self._markers.publish(marker)
                return
            rate.sleep()
        rospy.logwarn(
            'No subscribers to the marker publisher, did not publish marker.')

    # Pre-move_object position
    def pre_position_tool(self):
        goal = JointTrajectoryGoal()
        goal.trajectory.joint_names = [
            'l_shoulder_pan_joint',
            'l_shoulder_lift_joint',
            'l_upper_arm_roll_joint',
            'l_elbow_flex_joint',
            'l_forearm_roll_joint',
            'l_wrist_flex_joint',
            'l_wrist_roll_joint',
        ]
        point = JointTrajectoryPoint()
        point.positions = [
            0.1478368422400076,   # shoulder_pan_joint,
            0.5075741451910245,   # shoulder_lift_joint,
            0.07666276829324792,  # upper_arm_roll_joint,
            -2.1254967913712126,  # elbow_flex_joint,
            -3.2490637932,        # forearm_roll_joint,
            -1.6188519772530534,  # wrist_flex_joint,
            -0.08595766341572286  # wrist_roll_joint,
        ]
        point.time_from_start = rospy.Duration(2.0)
        goal.trajectory.points.append(point)

        arm = SimpleActionClient(
            'l_arm_controller/joint_trajectory_action',
            JointTrajectoryAction,
        )
        rospy.loginfo('Waiting for joint trajectory action server')
        arm.wait_for_server()
        return arm.send_goal_and_wait(goal)

    def get_yaw(self, bounding_box):
        # get euler angles and normalize
        orientation = bounding_box.pose.pose.orientation
        (yaw, pitch, roll) = tf.transformations.euler_from_quaternion(
            [orientation.w, orientation.x, orientation.y, orientation.z]
        )
        yaw = -yaw  # euler_from_quaternion gives it to us backwards
        # make sure it's in the range (-pi, pi]
        while yaw > math.pi:
            yaw -= 2 * math.pi
        while yaw <= -math.pi:
            yaw += 2 * math.pi

        rospy.loginfo('Sanity check: yaw should be the only nonzero')
        rospy.loginfo('roll {}, pitch {}, yaw {}'.format(roll, pitch, yaw))

        return yaw


    def get_box_ends(self, bounding_box):
        '''
        Get the corners of the bounding box from the given descriptor.
        return list of corners
           ends[0]: front corner of first end
           ends[1]: front corner of second end
           ends[2]: rear corner of first end
           ends[3]: rear corner of second end
        '''
 
        rospy.loginfo('Bounding box dimensions {} {} {}'
                      .format(bounding_box.dimensions.x,
                              bounding_box.dimensions.y,
                              bounding_box.dimensions.z)
                      )

        # First, figure out whether the y-axis points roughly along or opposite
        # the cluster's y-axis
        yaw = self.get_yaw(bounding_box)

        # get yaw sign
        y_sign = 1.0
        if yaw < 0:
            y_sign = -1.0

        # relative to the bounding box's pose
        ends_in_item_frame = [
            Point(
                x=x_sign * bounding_box.dimensions.x / 2.0,
                y=y_sign * bounding_box.dimensions.y / 2.0,
                z=bounding_box.dimensions.z / 2.0,
            )
            for y_sign in [y_sign, -y_sign]
            for x_sign in [1.0, -1.0]
        ]

        # transform to the cluster's pose
        position = bounding_box.pose.pose.position
        sin_yaw = math.sin(yaw)
        cos_yaw = math.cos(yaw)
        ends = [
            Point(
                x=point.x * cos_yaw - point.y * sin_yaw + position.x,
                y=point.x * sin_yaw + point.y * cos_yaw + position.y,
                z=point.z + position.z,
            )
            for point in ends_in_item_frame
        ]

        # visualize bounding box, pose, and ends
        viz.publish_bounding_box(
            self._markers,
            bounding_box.pose,
            bounding_box.dimensions.x,
            bounding_box.dimensions.y,
            bounding_box.dimensions.z,
            0.7, 0, 0, 0.25,
            IdTable.get_id('push_item_bounding_box')
        )
        viz.publish_pose(
            self._markers,
            bounding_box.pose,
            0, 0.7, 0, 0.5,
            IdTable.get_id('push_item_object_pose')
        )
        for idx, end in enumerate(ends):
            red = 0.0 if idx < 2 else 0.5
            green = 0.0 if idx < 2 else 0.5
            blue = 0.7 if idx < 2 else 0.0
            viz.publish_point(
                self._markers,
                bounding_box.pose.header.frame_id,
                end,
                red, green, blue, 0.5,
                IdTable.get_id('push_item_end_{}'.format(idx))
            )

        return ends


    @handle_service_exceptions(outcomes.SIMULATION_FAILURE)
    def execute(self, userdata):
        rospy.loginfo("Starting Simulation State!")


        bag = rosbag.Bag('bagfiles/notes_pose1.bag')
        for topic, msg, t in bag.read_messages(topics=['record']):
            msg.marker_boundingbox.header.stamp = rospy.Time().now()
            msg.marker_pointcloud.header.stamp = rospy.Time().now()
            self._publish(msg.marker_pointcloud)
            self._publish(msg.marker_boundingbox)
         
        bag.close()
        bounding_box = msg.boundingbox
        self.pre_position_tool()
        ends = self.get_box_ends(bounding_box)

        centroid = bounding_box.pose.pose.position
        frame = bounding_box.pose.header.frame_id

        # add the object we're pushing to the allowable collision matrix
        #self.add_allowable_collision_box(bounding_box)

        # position at which tip of tool makes contact with object, in cluster frame
        self.application_point = Point(0, 0, 0)


        while(True):
            
            tool_action = raw_input("enter the number of the tool action:")

            # Front side push
            if(tool_action == '1'):

                self.application_point.x = (centroid.x + target_end.x) / 2.0
                self.application_point.y = (centroid.y + target_end.y) / 2.0

                self.application_point.z = 0.09

                application_point = PointStamped(
                    header=Header(frame_id=frame),
                    point=self.application_point,
                )

                action = PushAway(
                    bounding_box,
                    application_point,
                    userdata,
                    **self.services
                )

                success = action.execute()

            # Front center push
            if(tool_action == '2'):

                target_end = ends[0]
                self.application_point.x = centroid.x 
                self.application_point.y = centroid.y 

                self.application_point.z = 0.09

                application_point = PointStamped(
                    header=Header(frame_id=frame),
                    point=self.application_point,
                )

                action = PushAway(
                    bounding_box,
                    application_point,
                    userdata,
                    **self.services
                )

                success = action.execute()

            if(tool_action == '3'):
                self.push_sideways_x_clearance = 0.01
                self.push_sideways_y_clearance = 0.02
                self.application_height = 0.09
                idx = 2

                front_corner = ends[idx - 2]
                rear_corner = ends[idx]

                push_direction_sign = 1.0
                if rear_corner.y - centroid.y < 0:
                    push_direction_sign = -1.0

                rospy.loginfo('Pushing sideways')

                # position at which tip of tool makes contact with object, in cluster frame
                self.application_point = Point(0, 0, 0)

                # Apply tool between target end and edge of bin
                self.application_point.x = rear_corner.x + self.push_sideways_x_clearance
                self.application_point.y = rear_corner.y + (push_direction_sign *
                                                            self.push_sideways_y_clearance)
                self.application_point.z = self.application_height
                action = PushSideways(bounding_box, self.application_point,
                                      userdata, **self.services)

                success = action.execute()

            if(tool_action == '4'):
                self.push_sideways_x_clearance = 0.01
                self.push_sideways_y_clearance = 0.02
                self.application_height = 0.09
                idx = 2

                front_corner = ends[idx - 2]
                rear_corner = ends[idx]

                push_direction_sign = 1.0
                if rear_corner.y - centroid.y < 0:
                    push_direction_sign = -1.0

                rospy.loginfo('Pushing sideways')

                # position at which tip of tool makes contact with object, in cluster frame
                self.application_point = Point(0, 0, 0)

                # Apply tool between target end and edge of bin
                self.application_point.x = centroid.x - 0.02
                self.application_point.y = rear_corner.y + (push_direction_sign *
                                                            self.push_sideways_y_clearance)
                self.application_point.z = self.application_height
                action = PushSideways(bounding_box, self.application_point,
                                      userdata, **self.services)

                success = action.execute()


            if(tool_action == '5'):
                self.push_down_offset = 0.02
                self.application_point.x = centroid.x 
                self.application_point.y = centroid.y
                self.application_point.z = centroid.z + 0.05
                action = PullForward(bounding_box, self.application_point,
                                     userdata, **self.services)
            
                success = action.execute()

            if(tool_action == '6'):

                self.push_down_offset = 0.02
                self.application_point.x = centroid.x 
                self.application_point.y = centroid.y
                self.application_point.z = centroid.z + 0.05
                action = TopSideways(bounding_box, self.application_point,
                                     userdata, **self.services)
            
                success = action.execute()

            self.pre_position_tool()

            if(tool_action == '-1'):
                break
        

        return outcomes.SIMULATION_SUCCESS
