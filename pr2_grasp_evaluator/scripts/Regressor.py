#!/usr/bin/env python

# Regression Stuff
from sklearn.datasets import fetch_olivetti_faces
from sklearn.utils.validation import check_random_state

from sklearn.ensemble import ExtraTreesRegressor
from sklearn.neighbors import KNeighborsRegressor
from sklearn.linear_model import LinearRegression
from sklearn.linear_model import RidgeCV
from sklearn.linear_model import BayesianRidge
from sklearn.linear_model import Lasso
from sklearn.linear_model import TheilSenRegressor


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
from pr2_pick_main import publish_gripper, publish_bounding_box, publish_cluster
from pr2_pick_perception.msg import Object, BoundingBox
from pr2_pick_perception.srv import CropShelf, CropShelfResponse, \
    DeleteStaticTransform, FindCentroid, LocalizeShelf, LocalizeShelfResponse, \
    SetStaticTransform, PlanarPrincipalComponents, GetItemDescriptor, ClassifyTargetItem
from pr2_gripper_grasp_planner_cluster.srv import SetPointClusterGraspParams, SetPointClusterGraspParamsRequest
from object_recognition_clusters.srv import FindClusterBoundingBox, FindClusterBoundingBoxRequest

class Regressor:
    def __init__(self, path):
        self._x_before = {}
        self._y_before = {}
        self._yaw_before = {}
        self._x_dim_before = {}
        self._y_dim_before = {}
        self._z_dim_before = {}

        self._x_after = {}
        self._y_after = {}
        self._yaw_after = {}
        self._x_dim_after = {}
        self._y_dim_after = {}
        self._z_dim_after = {}
        self.markers = rospy.Publisher('pr2_pick_visualization', Marker)
        self._path = path

        self.ESTIMATORS = {
            # "Extra trees": ExtraTreesRegressor(n_estimators=10, max_features=32,
            #                                    random_state=0),
            # "K-nn": KNeighborsRegressor(),
            "Linear Regression": LinearRegression(),
            # "Ridge": RidgeCV(), # Works
            # "Bayesian Ridge": BayesianRidge(),
            # "Theil-Sen Regressor": TheilSenRegressor(),
            # "Lasso": Lasso(), # Works
        }

        self._n_actions = 10

        self._max_depth = 3

        self.estimators = {}


    def get_bounding_box_diff(self, trial):
        action = trial.params.action.data
        #rospy.loginfo("Action: " + str(action))
        if not self._x_before.has_key(action):
            self._x_before[action] = []
            self._y_before[action] = []
            self._yaw_before[action] = []
            self._x_after[action] = []
            self._y_after[action] = []
            self._yaw_after[action] = []
            self._x_dim_before[action] = []
            self._x_dim_after[action] = []            
            self._y_dim_before[action] = []
            self._y_dim_after[action] = []            
            self._z_dim_before[action] = []
            self._z_dim_after[action] = []

        before_box = trial.before.boundingbox
        after_box = trial.after.boundingbox

        self._x_after[action].append(after_box.pose.pose.position.x)
        self._x_before[action].append(before_box.pose.pose.position.x)
        self._y_after[action].append(after_box.pose.pose.position.y)
        self._y_before[action].append(before_box.pose.pose.position.y)
        self._x_dim_before[action].append(before_box.dimensions.x)
        self._x_dim_after[action].append(after_box.dimensions.x)
        self._y_dim_before[action].append(before_box.dimensions.y)
        self._y_dim_after[action].append(after_box.dimensions.y)
        self._z_dim_before[action].append(before_box.dimensions.z)
        self._z_dim_after[action].append(after_box.dimensions.z)

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

        # rospy.loginfo("Before: {}".format(before_box))
        # rospy.loginfo("Before yaw: {}".format(yaw_before*180/math.pi))
        # rospy.loginfo("After: {}".format(after_box))
        # rospy.loginfo("After yaw: {}".format(yaw_after*180/math.pi))


        if (math.fabs(after_box.dimensions.x - before_box.dimensions.x) > 0.02) and (math.fabs(after_box.dimensions.y - before_box.dimensions.y) > 0.02):
            rospy.loginfo("Wrong axes")
            yaw_before -= math.pi/2.0
            y_dim_before = copy.copy(self._x_dim_before[action][-1])
            y_dim_after = copy.copy(self._x_dim_after[action][-1])
            x_dim_before = copy.copy(self._y_dim_before[action][-1])
            x_dim_after = copy.copy(self._y_dim_after[action][-1])
            self._x_dim_before[action][-1] = x_dim_before
            self._x_dim_after[action][-1] = x_dim_after
            self._y_dim_before[action][-1] =  y_dim_before
            self._y_dim_after[action][-1] =  y_dim_after
        
        if yaw_before < 0:
            if (math.fabs(yaw_after - (yaw_before + math.pi))) < (math.fabs(yaw_after - yaw_before)):
                rospy.loginfo("Flipped")
                yaw_before = (yaw_before + math.pi)
        else:
            if (math.fabs(yaw_after - (yaw_before - math.pi))) < (math.fabs(yaw_after - yaw_before)):
                rospy.loginfo("Flipped")
                yaw_before = (yaw_before - math.pi)

        rospy.loginfo("Yaw diff: {}".format(yaw_diff*180/math.pi))

        self._yaw_before[action].append(copy.copy(yaw_before))
        self._yaw_after[action].append(copy.copy(yaw_after))


    def get_test_data(self, trial):
        action = trial.params.action.data
        #rospy.loginfo("Action: " + str(action))

        x_before = {}
        y_before = {}
        _yaw_before = {}
        x_after = {}
        y_after = {}
        _yaw_after = {}

        if not x_before.has_key(action):
            x_before[action] = []
            y_before[action] = []
            _yaw_before[action] = []
            x_after[action] = []
            y_after[action] = []
            _yaw_after[action] = []

        before_box = trial.before.boundingbox
        after_box = trial.after.boundingbox

        x_after[action].append(after_box.pose.pose.position.x)
        x_before[action].append(before_box.pose.pose.position.x)
        y_after[action].append(after_box.pose.pose.position.y)
        y_before[action].append(before_box.pose.pose.position.y)

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

        # rospy.loginfo("Before: {}".format(before_box))
        # rospy.loginfo("Before yaw: {}".format(yaw_before*180/math.pi))
        # rospy.loginfo("After: {}".format(after_box))
        # rospy.loginfo("After yaw: {}".format(yaw_after*180/math.pi))


        if (math.fabs(after_box.dimensions.x - before_box.dimensions.x) > 0.02) and (math.fabs(after_box.dimensions.y - before_box.dimensions.y) > 0.02):
            rospy.loginfo("Wrong axes")
            yaw_before -= math.pi/2.0

        
        if yaw_before < 0:
            if (math.fabs(yaw_after - (yaw_before + math.pi))) < (math.fabs(yaw_after - yaw_before)):
                rospy.loginfo("Flipped")
                yaw_before = (yaw_before + math.pi)
        else:
            if (math.fabs(yaw_after - (yaw_before - math.pi))) < (math.fabs(yaw_after - yaw_before)):
                rospy.loginfo("Flipped")
                yaw_before = (yaw_before - math.pi)

        rospy.loginfo("Yaw diff: {}".format(yaw_diff*180/math.pi))

        _yaw_before[action].append(copy.copy(yaw_before))
        _yaw_after[action].append(copy.copy(yaw_after))

        return x_before, x_after, y_before, y_after, _yaw_before, _yaw_after

        # self._action_names.append(trial.params.action)
        # self._item_names.append(trial.params.item_name)

    def train(self):
        file_list = glob.glob(self._path + '/*.bag')
    
        for file_name in file_list:
            if not file_name[-13:] == 'evaluated.bag':
                continue
            # rospy.loginfo("Current file: " + file_name)
            bag = rosbag.Bag(file_name)
            for topic, trial_msg, t in bag.read_messages():
                self.get_bounding_box_diff(trial_msg)

            bag.close()

        for action in self._x_before.keys():

            self.estimators[action] = LinearRegression()

            train_x_before = self._x_before[action]
            train_y_before = self._y_before[action]
            train_yaw_before = self._yaw_before[action]
            train_x_dim_before = self._x_dim_before[action]
            train_y_dim_before = self._y_dim_before[action]
            train_z_dim_before = self._z_dim_before[action]

            train_x_after = self._x_after[action]
            train_y_after = self._y_after[action]
            train_yaw_after = self._yaw_after[action]
            train_x_dim_after = self._x_dim_after[action]
            train_y_dim_after = self._y_dim_after[action]
            train_z_dim_after = self._z_dim_after[action]

            self.estimators[action].fit(zip(train_x_before, train_y_before, 
                                            train_yaw_before, train_x_dim_before, 
                                            train_y_dim_before, train_z_dim_before), 
                                        zip(train_x_after, train_y_after, 
                                            train_yaw_after, train_x_dim_after, 
                                            train_y_dim_after, train_z_dim_after))            


    def estimate(self):
        # plot x_diffs

        file_name = raw_input("Please enter the path to a test bag file: ")
      
        # rospy.loginfo("Current file: " + file_name)
        bag = rosbag.Bag(file_name)
        trial = None
        for topic, trial_msg, t in bag.read_messages():
            _x_before, _x_after, _y_before, _y_after, _yaw_before, _yaw_after = self.get_test_data(trial_msg)
            trial = trial_msg

        bag.close()

        action = _x_before.keys()[0]

        train_x_before = self._x_before[action]
        train_y_before = self._y_before[action]
        train_yaw_before = self._yaw_before[action]

        train_x_after = self._x_after[action]
        train_y_after = self._y_after[action]
        train_yaw_after = self._yaw_after[action]

        test_x_before = _x_before[action]
        test_y_before = _y_before[action]
        test_yaw_before = _yaw_before[action]

        test_x_after = _x_after[action]
        test_y_after = _y_after[action]
        test_yaw_after = _yaw_after[action]

        rospy.loginfo("Train x before: {}".format(train_x_before))
        rospy.loginfo("Train y before: {}".format(train_y_before))
        rospy.loginfo("Train yaw before: {}".format(train_yaw_before))
        rospy.loginfo("Test x before: {}".format(test_x_before))
        rospy.loginfo("Test y before: {}".format(test_y_before))
        rospy.loginfo("Test yaw before: {}".format(test_yaw_before))
        rospy.loginfo("Test x after: {}".format(test_x_after))
        rospy.loginfo("Test y after: {}".format(test_y_after))
        rospy.loginfo("Test yaw after: {}".format(test_yaw_after))

        # Fit estimators
        ESTIMATORS = {
            # "Extra trees": ExtraTreesRegressor(n_estimators=10, max_features=32,
            #                                    random_state=0),
            # "K-nn": KNeighborsRegressor(),
            "Linear Regression": LinearRegression(),
            "Ridge": RidgeCV(),
            # "Bayesian Ridge": BayesianRidge(),
            # "Theil-Sen Regressor": TheilSenRegressor(),
            "Lasso": Lasso(),
        }

        y_test_predict = dict()
        for name, estimator in ESTIMATORS.items():
            estimator.fit(zip(train_x_before, train_y_before, train_yaw_before), zip(train_x_after, train_y_after, train_yaw_after))
            y_test_predict[name] = estimator.predict(zip(test_x_before, test_y_before, test_yaw_before))

        rospy.loginfo("Predictions: {}".format(y_test_predict))
        rospy.loginfo("Pose: {}".format(trial.before.boundingbox.pose))

        # First publish original bounding box

        publish_bounding_box(self.markers, trial.before.boundingbox.pose, 
                (trial.before.boundingbox.dimensions.x), 
                (trial.before.boundingbox.dimensions.y), 
                (trial.before.boundingbox.dimensions.z),
                1.0, 1.0, 0.0, 0.5, 20)

        # And true result

        publish_bounding_box(self.markers, trial.after.boundingbox.pose, 
                (trial.after.boundingbox.dimensions.x), 
                (trial.after.boundingbox.dimensions.y), 
                (trial.after.boundingbox.dimensions.z),
                0.0, 1.0, 0.0, 0.5, 21)

        # Now one for each estimator
        colour = [{"r": 1.0, "g": 0.0, "b": 0.0}, {"r": 0.0, "g": 0.0, "b": 1.0}, 
                    {"r": 1.0, "g": 0.0, "b": 1.0}, {"r": 0.0, "g": 1.0, "b": 1.0}]
        for j, est in enumerate(sorted(ESTIMATORS)):
                x = y_test_predict[est][0][0]
                y = y_test_predict[est][0][1]
                yaw = y_test_predict[est][0][2]


                pose = PoseStamped()
                pose.header.frame_id = "bin_K"
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = trial.before.boundingbox.pose.pose.position.z


                quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
                #type(pose) = geometry_msgs.msg.Pose
                pose.pose.orientation.x = quaternion[0]
                pose.pose.orientation.y = quaternion[1]
                pose.pose.orientation.z = quaternion[2]
                pose.pose.orientation.w = quaternion[3]

                publish_bounding_box(self.markers, pose, 
                    (trial.after.boundingbox.dimensions.x), 
                    (trial.after.boundingbox.dimensions.y), 
                    (trial.after.boundingbox.dimensions.z),
                    colour[j]["r"], colour[j]["g"], colour[j]["b"], 0.5, j + 30)

    def sanity_check(self):
        # plot x_diffs

        action = self._x_before.keys()[0]

        train_x_before = self._x_before[action]
        train_y_before = self._y_before[action]
        train_yaw_before = self._yaw_before[action]

        train_x_after = self._x_after[action]
        train_y_after = self._y_after[action]
        train_yaw_after = self._yaw_after[action]


        # rospy.loginfo("Train x before: {}".format(train_x_before))
        # rospy.loginfo("Train y before: {}".format(train_y_before))
        # rospy.loginfo("Train yaw before: {}".format(train_yaw_before))
        # rospy.loginfo("Test x before: {}".format(test_x_before))
        # rospy.loginfo("Test y before: {}".format(test_y_before))
        # rospy.loginfo("Test yaw before: {}".format(test_yaw_before))
        # rospy.loginfo("Test x after: {}".format(test_x_after))
        # rospy.loginfo("Test y after: {}".format(test_y_after))
        # rospy.loginfo("Test yaw after: {}".format(test_yaw_after))

        # Fit estimators
        ESTIMATORS = {
            # "Extra trees": ExtraTreesRegressor(n_estimators=10, max_features=32,
            #                                    random_state=0),
            # "K-nn": KNeighborsRegressor(),
            "Linear Regression": LinearRegression(),
            "Ridge": RidgeCV(),
            # "Bayesian Ridge": BayesianRidge(),
            # "Theil-Sen Regressor": TheilSenRegressor(),
            "Lasso": Lasso(),
        }

        y_test_predict = dict()
        for name, estimator in ESTIMATORS.items():
            estimator.fit(zip(train_x_before, train_y_before, train_yaw_before), zip(train_x_after, train_y_after, train_yaw_after))
            y_test_predict[name] = estimator.predict(zip(train_x_before, train_y_before, train_yaw_before))

        rospy.loginfo("Predictions: {}".format(y_test_predict))
        # rospy.loginfo("Pose: {}".format(trial.before.boundingbox.pose))

        err_x = 0
        err_y = 0
        err_yaw = 0


        for i in range(len(train_x_before)):
            err_x += math.fabs(train_x_after[i] - y_test_predict["Linear Regression"][i][0])
            err_y += math.fabs(train_y_after[i]-  y_test_predict["Linear Regression"][i][1])
            err_yaw += math.fabs(train_yaw_after[i] - y_test_predict["Linear Regression"][i][2])

        rospy.loginfo("Avg x error: {}".format(err_x/len(train_x_before)))
        rospy.loginfo("Avg y error: {}".format(err_y/len(train_x_before)))
        rospy.loginfo("Avg yaw error: {}".format(err_yaw/len(train_x_before)))

    def predict(self, action, bounding_box):

        test_x = [bounding_box.pose.pose.position.x]
        test_y = [bounding_box.pose.pose.position.y]

        quaternion = (
            bounding_box.pose.pose.orientation.x,
            bounding_box.pose.pose.orientation.y,
            bounding_box.pose.pose.orientation.z,
            bounding_box.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)

        yaw = euler[2]

        test_yaw = [yaw]

        test_x_dim = [bounding_box.dimensions.x]
        test_y_dim = [bounding_box.dimensions.y]
        test_z_dim = [bounding_box.dimensions.z]

        # Fit estimators
        ESTIMATORS = {
            # "Extra trees": ExtraTreesRegressor(n_estimators=10, max_features=32,
            #                                    random_state=0),
            # "K-nn": KNeighborsRegressor(),
            "Linear Regression": LinearRegression(),
            "Ridge": RidgeCV(),
            # "Bayesian Ridge": BayesianRidge(),
            # "Theil-Sen Regressor": TheilSenRegressor(),
            "Lasso": Lasso(),
        }

        # y_test_predict = dict()
        
        estimator = self.estimators[action]
        y_test_predict = estimator.predict(zip(test_x, test_y, test_yaw, test_x_dim, test_y_dim, test_z_dim))

        rospy.loginfo("Predictions: {}".format(y_test_predict))
        rospy.loginfo("Pose: {}".format(bounding_box.pose))

        # First publish original bounding box

        publish_bounding_box(self.markers, bounding_box.pose, 
                (bounding_box.dimensions.x), 
                (bounding_box.dimensions.y), 
                (bounding_box.dimensions.z),
                1.0, 1.0, 0.0, 0.5, 20)

        # And true result

        # publish_bounding_box(self.markers, bounding_box.pose, 
        #         (bounding_box.dimensions.x), 
        #         (bounding_box.dimensions.y), 
        #         (bounding_box.dimensions.z),
        #         0.0, 1.0, 0.0, 0.5, 21)

        # Now one for each estimator
        colour = {"r": 1.0, "g": 0.0, "b": 0.0}


        x = y_test_predict[0][0]
        y = y_test_predict[0][1]
        yaw = y_test_predict[0][2]


        pose = PoseStamped()
        pose.header.frame_id = "bin_K"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = bounding_box.pose.pose.position.z


        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        #type(pose) = geometry_msgs.msg.Pose
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]

        publish_bounding_box(self.markers, pose, 
            (y_test_predict[0][3]), 
            (y_test_predict[0][4]), 
            (y_test_predict[0][5]),
            colour["r"], colour["g"], colour["b"], 0.5, 30)

        return x, y, yaw, y_test_predict[0][3], y_test_predict[0][4], y_test_predict[0][5]


if __name__ == '__main__':

    rospy.init_node('trial_analyser')
    path = raw_input("Please enter the path to folder with the bag files: ")

    ta = Regressor(path)

    

    ta.sanity_check()