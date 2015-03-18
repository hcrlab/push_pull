#!/usr/bin/env python

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--interactive", action="store_true")
parser.add_argument("--position_only", action="store_true")
args = parser.parse_args()


from trajopt_test.srv import *
import os, os.path as osp
import subprocess
import shlex
import openravepy
import trajoptpy
import cloudprocpy
import trajoptpy.make_kinbodies as mk
import json
import numpy as np
import trajoptpy.kin_utils as ku
from joint_states_listener.srv import ReturnJointStates
#from point_cloud_listener.srv import ReturnPointCloud
import time
import sys
from sensor_msgs.msg import JointState, PointCloud2
import threading
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import actionlib
import rospy
import tf
import random

class Arm:
    def __init__(self, arm_name):
        #arm_name should be l_arm or r_arm
        self.name = arm_name
        self.jta = actionlib.SimpleActionClient('/'+arm_name+'_controller/joint_trajectory_action',
                        JointTrajectoryAction)
        rospy.loginfo('Waiting for joint trajectory action')
        self.jta.wait_for_server()
        rospy.loginfo('Found joint trajectory action!')

    def move(self, waypoints):
        goal = JointTrajectoryGoal()
        char = self.name[0] #either 'r' or 'l'
        goal.trajectory.joint_names = [char+'_shoulder_pan_joint',
                   char+'_shoulder_lift_joint',
                   char+'_upper_arm_roll_joint',
                   char+'_elbow_flex_joint',
                   char+'_forearm_roll_joint',
                   char+'_wrist_flex_joint',
                   char+'_wrist_roll_joint']
        time_offset = 0
        for waypoint in waypoints:
            point = JointTrajectoryPoint()
            point.positions = waypoint
            point.time_from_start = rospy.Duration(1 + time_offset*1)
            goal.trajectory.points.append(point)
            time_offset = time_offset + 1
            
        self.jta.send_goal_and_wait(goal)


def generate_mesh(cloud, T_w_k):
    #cloud = cloudprocpy.medianFilter(cloud, 15, .05) # smooth out the depth image
    cloud = remove_floor(cloud, T_w_k) # remove points with low height (in world frame)
    big_mesh = cloudprocpy.meshOFM(cloud, 3, .1) # use pcl OrganizedFastMesh to make mesh
    simple_mesh = cloudprocpy.quadricSimplifyVTK(big_mesh, .02) # decimate mesh with VTK function
    return simple_mesh

def get_xyz_world_frame(cloud, T_w_k):
    xyz1 = cloud.to2dArray()
    xyz1[:,3] = 1
    return xyz1.dot(T_w_k.T)[:,:3]


def remove_floor(cloud, T_w_k):
    notfloor = get_xyz_world_frame(cloud, T_w_k)[:,2] > .1
    cloud = cloudprocpy.maskFilter(cloud, notfloor, True)
    return cloud


def navigate(req):
    #print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b))

    print "Navigating!"

    #rospy.init_node('trajopt_no_pcl')
    env = openravepy.Environment()
    env.StopSimulation()
    env.Load("robots/pr2-beta-static.zae")
    #env.Load("../data/table.xml")
    robot = env.GetRobots()[0]
    #env.SetViewer('qtcoin')

    trajoptpy.SetInteractive(args.interactive) # pause every iteration, until you press 'p'. Press escape to disable further plotting

    rospy.wait_for_service("return_joint_states")

    joint_names = [];
    for j in robot.GetJoints():
        joint_names.append(j.GetName())
 
    try:
        s = rospy.ServiceProxy("return_joint_states", ReturnJointStates)
        resp = s(joint_names)
    except rospy.ServiceException, e:
        print "error when calling return_joint_states: %s"%e
        sys.exit(1)
    for (ind, joint_name) in enumerate(joint_names):
        if(not resp.found[ind]):
            print "joint %s not found!"%joint_name

    
    listener = tf.TransformListener()
    listener.waitForTransform("/base_footprint","/head_mount_kinect_rgb_optical_frame", rospy.Time(0),rospy.Duration(1))
    (x,y,z), (qx,qy,qz,qw) = listener.lookupTransform("/base_footprint","/head_mount_kinect_rgb_optical_frame", rospy.Time(0))
    T_w_k = openravepy.matrixFromPose([qw,qx,qy,qz,x,y,z])

    dof_vals = robot.GetDOFValues()

        #time.sleep(5)

    print "Before making mesh"
    
    if req.new_cloud:
        grabber=cloudprocpy.CloudGrabber()


        xyzrgb = None
        point_cloud = None
        xyzrgb = grabber.getXYZRGB()
    

     	xyzrgb.save("cloud.pcd")
    point_cloud = cloudprocpy.readPCDXYZ("cloud.pcd")
    

    mesh = generate_mesh(point_cloud, T_w_k)
    mesh_body = mk.create_trimesh(env, get_xyz_world_frame(mesh.getCloud(), T_w_k), np.array(mesh.getFaces()), name="simple_mesh")
    mesh_body.SetUserData("bt_use_trimesh", True) # Tell collision checker to use the trimesh rather than the convex hull of it
    print "After making mesh"

    
    joint_start = resp.position

    print "Joint start: " + str(joint_start)
    robot.SetDOFValues(joint_start) # robot.GetManipulator('rightarm').GetArmIndices())
    #time.sleep(3)

    quat_target = [req.goal.pose.orientation.w, req.goal.pose.orientation.x, req.goal.pose.orientation.y, req.goal.pose.orientation.z] # wxyz
    xyz_target = [req.goal.pose.position.x, req.goal.pose.position.y, req.goal.pose.position.z]
    hmat_target = openravepy.matrixFromPose( np.r_[quat_target, xyz_target] )

    # BEGIN ik
    init_joint_target = None
    manip = robot.GetManipulator("rightarm")
    init_joint_target = ku.ik_for_link(hmat_target, manip, "r_gripper_tool_frame")

    tries = 0
    """
    while ((init_joint_target == None) and (tries < 100)):
        tries = tries + 1
        xyz_target[0] = xyz_target[0] + random.uniform(-0.05, 0.05)
        xyz_target[1] = xyz_target[1] + random.uniform(-0.05, 0.05)
        xyz_target[2] = xyz_target[2] + random.uniform(-0.05, 0.05)
        init_joint_target = ku.ik_for_link(hmat_target, manip, "r_gripper_tool_frame",
            filter_options = openravepy.IkFilterOptions.CheckEnvCollisions)
    """

    print "Final target: " + str(xyz_target) + " Took " + str(tries) + " tries."
    # END ik
    print "Target: " + str(init_joint_target)

    if init_joint_target == None:
        return TrajoptNavigateResponse(False)

    #robot.SetDOFValues(init_joint_target, robot.GetManipulator('rightarm').GetArmIndices())
    init_joint_target = ku.ik_for_link(hmat_target, manip, "r_gripper_tool_frame",
            filter_options = openravepy.IkFilterOptions.CheckEnvCollisions)

    #time.sleep(3)

    # NOw set the start position again to start planning
    robot.SetDOFValues(joint_start)

    request = {
      "basic_info" : {
        "n_steps" : 10,
        "manip" : "rightarm", # see below for valid values
        "start_fixed" : True # i.e., DOF values at first timestep are fixed based on current robot state
      },
      "costs" : [
      {
        "type" : "joint_vel", # joint-space velocity cost
        "params": {"coeffs" : [1]} # a list of length one is automatically expanded to a list of length n_dofs
      },
      {
        "type" : "collision",
        "name" :"cont_coll", # shorten name so printed table will be prettier
        "params" : {
          "continuous" : True,
          "coeffs" : [20], # penalty coefficients. list of length one is automatically expanded to a list of length n_timesteps
          "dist_pen" : [0.025] # robot-obstacle distance that penalty kicks in. expands to length n_timesteps
        }
      },
      {
        "type" : "collision",
        "name" : "disc_coll", # For handling self-collision as continuous cost cannot handle it
        "params" : {"coeffs" : [20],"dist_pen" : [0.025], "continuous":False}
      }
      ],
      "constraints" : [
      # BEGIN pose_constraint
      {
        "type" : "pose", 
        "params" : {"xyz" : xyz_target, 
                    "wxyz" : quat_target, 
                    "link": "r_gripper_tool_frame",
                    "timestep" : 9
                    }
                     
      }
      # END pose_constraint
      ],
      # BEGIN init
      "init_info" : {
          "type" : "straight_line", # straight line in joint space.
          "endpoint" : init_joint_target.tolist() # need to convert numpy array to list
      }
      # END init
    }

    if args.position_only: request["constraints"][0]["params"]["rot_coeffs"] = [0,0,0]

    s = json.dumps(request) # convert dictionary into json-formatted string

    print "Before constructing problem"
    prob = trajoptpy.ConstructProblem(s, env) # create object that stores optimization problem
    print "Constructing problem"
    result = trajoptpy.OptimizeProblem(prob) # do optimization
    print result

    from trajoptpy.check_traj import traj_is_safe
    prob.SetRobotActiveDOFs() # set robot DOFs to DOFs in optimization problem
    #assert traj_is_safe(result.GetTraj(), robot) # Check that trajectory is collision free

    # Replay the trajectory on OpenRAVE
    #time.sleep(2)
    #for position in result.GetTraj():
    #    robot.SetDOFValues(position, robot.GetManipulator('rightarm').GetArmIndices())
    #    time.sleep(1)

    # Now we'll check to see that the final constraint was satisfied
    #robot.SetActiveDOFValues(result.GetTraj()[-1])
    #posevec = openravepy.poseFromMatrix(robot.GetLink("r_gripper_tool_frame").GetTransform())
    #quat, xyz = posevec[0:4], posevec[4:7]

    #quat *= np.sign(quat.dot(quat_target))
    #if args.position_only:
    #    assert (quat - quat_target).max() > 1e-3
    #else:
    #    assert (quat - quat_target).max() < 1e-3

    sum_costs = 0
    for cost in result.GetCosts():
        sum_costs = sum_costs + cost[1]
    if sum_costs < 70:
        arm = Arm('r_arm')
        arm.move(result.GetTraj())
    else:
        print "Cost of " + str(sum_costs) + " is too big."
        return TrajoptNavigateResponse(False)

    return TrajoptNavigateResponse(True)
    

if __name__ == "__main__":
    rospy.init_node('trajopt_server')
    s = rospy.Service('trajopt_navigate', TrajoptNavigate, navigate)
    rospy.spin()
