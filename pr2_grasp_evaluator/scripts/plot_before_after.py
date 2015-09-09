#!/usr/bin/env python

# Python Stuff
import glob
import copy
import itertools
import numpy as np
import matplotlib.pyplot as plt
import math

# ROS Stuff
import actionlib
from actionlib import SimpleActionClient
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, TransformStamped, \
    Point, PointStamped, Vector3, Transform
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from manipulation_msgs.srv import GraspPlanning, GraspPlanningRequest
from manipulation_msgs.msg import GraspPlanningAction, GraspPlanningGoal
import moveit_commander
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene, GetPositionFK, GetPositionIK
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
import rosbag
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
import tf
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker

# Local Stuff
from joint_states_listener.srv import ReturnJointStates
from pr2_pick_contest.msg import Trial, Record
from pr2_pick_contest.srv import GetItems, SetItems, GetTargetItems
from pr2_pick_contest.srv import LookupItem
from pr2_pick_perception.msg import Object, BoundingBox
from pr2_pick_perception.srv import CropShelf, CropShelfResponse, \
    DeleteStaticTransform, FindCentroid, LocalizeShelf, LocalizeShelfResponse, \
    SetStaticTransform, PlanarPrincipalComponents, GetItemDescriptor, ClassifyTargetItem
from pr2_gripper_grasp_planner_cluster.srv import SetPointClusterGraspParams, SetPointClusterGraspParamsRequest
from object_recognition_clusters.srv import FindClusterBoundingBox, FindClusterBoundingBoxRequest

class TrialAnalyser:
    def __init__(self):
        self._x_diffs = {}
        self._y_diffs = {}
        self._yaw_diffs = {}
        self._graspable_before = []
        self._graspable_after = []
        self._increased_grasps = []


    def get_bounding_box_diff(self, trial):
        action = trial.params.action.data
        #rospy.loginfo("Action: " + str(action))
        if not self._x_diffs.has_key(action):
            self._x_diffs[action] = []
            self._y_diffs[action] = []
            self._yaw_diffs[action] = []

        before_box = trial.before.boundingbox
        after_box = trial.after.boundingbox

        self._x_diffs[action].append(after_box.pose.pose.position.x - before_box.pose.pose.position.x)
        self._y_diffs[action].append(after_box.pose.pose.position.y - before_box.pose.pose.position.y)

        q_before = (
            before_box.pose.pose.orientation.x,
            before_box.pose.pose.orientation.y,
            before_box.pose.pose.orientation.z,
            before_box.pose.pose.orientation.w)

        q_after = (
            after_box.pose.pose.orientation.x,
            after_box.pose.pose.orientation.y,
            after_box.pose.pose.orientation.z,
            after_box.pose.pose.orientation.w)
        euler_before = tf.transformations.euler_from_quaternion(q_before)
        euler_after = tf.transformations.euler_from_quaternion(q_after)

        yaw_before = euler_before[2]
        yaw_after = euler_after[2]

        yaw_diff = yaw_after - yaw_before

        rospy.loginfo("Before: {}".format(before_box))
        rospy.loginfo("Before yaw: {}".format(yaw_before*180/math.pi))
        rospy.loginfo("After: {}".format(after_box))
        rospy.loginfo("After yaw: {}".format(yaw_after*180/math.pi))

        if (math.fabs(after_box.dimensions.x - before_box.dimensions.x) > 0.02) and (math.fabs(after_box.dimensions.y - before_box.dimensions.y) > 0.02):
            rospy.loginfo("Wrong axes")
            if yaw_diff > 0:
                yaw_diff-=math.pi/2
            else:
                yaw_diff+=math.pi/2

        else:
            if yaw_before < 0:
                if (math.fabs(yaw_after - (yaw_before + math.pi))) < (math.fabs(yaw_after - yaw_before)):
                    rospy.loginfo("Flipped")
                    yaw_diff = yaw_after - (yaw_before + math.pi)
            else:
                if (math.fabs(yaw_after - (yaw_before - math.pi))) < (math.fabs(yaw_after - yaw_before)):
                    rospy.loginfo("Flipped")
                    yaw_diff = yaw_after - (yaw_before - math.pi)

        rospy.loginfo("Yaw diff: {}".format(yaw_diff*180/math.pi))

        self._yaw_diffs[action].append(copy.copy(yaw_diff))

        # self._action_names.append(trial.params.action)
        # self._item_names.append(trial.params.item_name)            


    def plot(self):
        # plot x_diffs

        for key in self._x_diffs:
            self._x_diffs[key] = [x * 100 for x in self._x_diffs[key]]
            self._y_diffs[key] = [x * 100 for x in self._y_diffs[key]]
            self._yaw_diffs[key] = [x * 180/math.pi for x in self._yaw_diffs[key]]


        actions = self._x_diffs.keys()

        N = len(actions)
        x_means = []
        x_stds = []
        y_means = []
        y_stds = []
        yaw_means = []
        yaw_stds = []

        rospy.loginfo("Yaw info: {}".format(self._yaw_diffs))

        for action in actions:
            x_mean = np.mean(self._x_diffs[action])
            x_std = np.std(self._x_diffs[action])

            x_means.append(copy.copy(x_mean))
            x_stds.append(copy.copy(x_std))

            y_mean = np.mean(self._y_diffs[action])
            y_std = np.std(self._y_diffs[action])

            y_means.append(copy.copy(y_mean))
            y_stds.append(copy.copy(y_std))

            yaw_mean = np.mean(self._yaw_diffs[action])
            yaw_std = np.std(self._yaw_diffs[action])

            yaw_means.append(copy.copy(yaw_mean))
            yaw_stds.append(copy.copy(yaw_std))

        ind = np.arange(N)  # the x locations for the groups
        width = 0.35       # the width of the bars

        fig, ax = plt.subplots()
        rects1 = ax.bar(ind, x_means, width, color='r', yerr=x_std)
        rects2 = ax.bar(ind+width, y_means, width, color='g', yerr=y_std)
        rects3 = ax.bar(ind+width+width, yaw_means, width, color='y', yerr=yaw_std)

        # add some text for labels, title and axes ticks
        ax.set_ylabel('Mean Displacement')
        ax.set_title('Crayons')
        ax.set_xticks(ind+width)
        ax.set_xticklabels( actions )

        ax.legend( (rects1[0], rects2[0], rects3[0]), ('X', 'Y', 'Yaw') )

        # for rects in [rects1, rects2, rects3]:

        #     # attach some text labels
        #     for rect in rects:
        #         height = rect.get_height()
        #         ax.text(rect.get_x()+rect.get_width()/2., 1.05*height, '%d'%int(height),
        #                 ha='center', va='bottom')

        # self.autolabel(rects1)
        # self.autolabel(rects2)
        # self.autolabel(rects3)


        plt.show()

    def get_num_grasps(self, trial_msg):
        if len(trial_msg.before.grasps)  > 0:
            self._graspable_before.append(trial_msg)
        if len(trial_msg.after.grasps) > 0:
            self._graspable_after.append(trial_msg)
        if len(trial_msg.after.grasps) > len(trial_msg.before.grasps):
            self._increased_grasps.append(trial_msg)

    def get_configuration_info(self, msgs):
        data_string = " "
        for trial_msg in msgs:
            data_string = data_string + "\n Item: {}, \t Position: {}, \t Orientation: {}, \t Action: {}".format(trial_msg.params.item_name.data,
                                                                                                                 trial_msg.params.position.data,
                                                                                                                 trial_msg.params.orientation.data,
                                                                                                                 trial_msg.params.action.data)
        return data_string

    def print_info(self):
        rospy.loginfo("Initially graspable trials: {}, {}".format(len(self._graspable_before), self.get_configuration_info(self._graspable_before)))
        rospy.loginfo("Trials graspable after: {}, {}".format(len(self._graspable_after), self.get_configuration_info(self._graspable_after)))
        self._became_graspable = []
        for item in self._graspable_after:
            if not item in self._graspable_before:
                self._became_graspable.append(item)
        rospy.loginfo("Not graspable before, graspable after: {}, {}".format(len(self._became_graspable), self.get_configuration_info(self._became_graspable)))
        rospy.loginfo("Increased number grasps after action: {}, {}".format(len(self._increased_grasps), self.get_configuration_info(self._increased_grasps)))




if __name__ == '__main__':
    rospy.init_node('trial_analyser')

    ta = TrialAnalyser()

    path = raw_input("Please enter the path to folder with the bag files: ")
    file_list = glob.glob( path + '/*.bag')
    ta.total_num_trials = 0
    for file_name in file_list:
        if file_name[-13:] != 'evaluated.bag':
            continue
        # rospy.loginfo("Current file: " + file_name)
        bag = rosbag.Bag(file_name)
        for topic, trial_msg, t in bag.read_messages():
            ta.total_num_trials+=1
            ta.get_bounding_box_diff(trial_msg)
            ta.get_num_grasps(trial_msg)

        bag.close()


    ta.print_info()
    ta.plot()
    # rospy.spin()
