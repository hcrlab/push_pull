import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--interactive", action="store_true")
parser.add_argument("--position_only", action="store_true")
args = parser.parse_args()


import os, os.path as osp

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
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import actionlib
import rospy
import tf
import random
point_cloud = None

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
            point.time_from_start = rospy.Duration(1 + time_offset)
            goal.trajectory.points.append(point)
            time_offset = time_offset + 1
            
        self.jta.send_goal_and_wait(goal)

#callback function: when a joint_states message arrives, save the values
def point_cloud_callback(msg):
   #self.lock.acquire()
   point_cloud = msg.data
   #self.lock.release()

def generate_mesh(cloud):
    cloud = cloudprocpy.fastBilateralFilter(cloud, 15, .05) # smooth out the depth image
    cloud = remove_floor(cloud) # remove points with low height (in world frame)
    big_mesh = cloudprocpy.meshOFM(cloud, 3, .1) # use pcl OrganizedFastMesh to make mesh
    simple_mesh = cloudprocpy.quadricSimplifyVTK(big_mesh, .02) # decimate mesh with VTK function
    return simple_mesh

def get_xyz_world_frame(cloud):
    xyz1 = cloud.to2dArray()
    xyz1[:,3] = 1
    return xyz1.dot(T_w_k.T)[:,:3]


def remove_floor(cloud):
    notfloor = get_xyz_world_frame(cloud)[:,2] > .1
    cloud = cloudprocpy.maskFilter(cloud, notfloor, True)
    return cloud

 
if __name__ == '__main__':

    rospy.init_node('trajopt_no_pcl')
    env = openravepy.Environment()
    env.StopSimulation()
    env.Load("robots/pr2-beta-static.zae")
    #env.Load("../data/table.xml")
    robot = env.GetRobots()[0]
    env.SetViewer('qtcoin')

    trajoptpy.SetInteractive(args.interactive) # pause every iteration, until you press 'p'. Press escape to disable further plotting

    rospy.wait_for_service("return_joint_states")
    #rospy.wait_for_service("return_point_cloud")
    #rospy.Subscriber('/head_mount_kinect/depth_registered/points', PointCloud2, point_cloud_callback)

    joint_names = [];
    for j in robot.GetJoints():
        joint_names.append(j.GetName())
 
  #joint_names = ['fl_caster_rotation_joint', 'fl_caster_l_wheel_joint', 'fl_caster_r_wheel_joint', 'fr_caster_rotation_joint', 'fr_caster_l_wheel_joint', 'fr_caster_r_wheel_joint', 'bl_caster_rotation_joint', 'bl_caster_l_wheel_joint', 'bl_caster_r_wheel_joint', 'br_caster_rotation_joint', 'br_caster_l_wheel_joint', 'br_caster_r_wheel_joint', 'torso_lift_joint', 'torso_lift_motor_screw_joint', 'head_pan_joint', 'head_tilt_joint', 'laser_tilt_mount_joint', 'r_upper_arm_roll_joint', 'r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_forearm_roll_joint', 'r_elbow_flex_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint', 'r_gripper_joint', 'r_gripper_l_finger_joint', 'r_gripper_r_finger_joint', 'r_gripper_r_finger_tip_joint', 'r_gripper_l_finger_tip_joint', 'r_gripper_motor_screw_joint', 'r_gripper_motor_slider_joint', 'l_upper_arm_roll_joint', 'l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_forearm_roll_joint', 'l_elbow_flex_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint', 'l_gripper_joint', 'l_gripper_l_finger_joint', 'l_gripper_r_finger_joint', 'l_gripper_r_finger_tip_joint', 'l_gripper_l_finger_tip_joint', 'l_gripper_motor_screw_joint', 'l_gripper_motor_slider_joint'] 
    try:
        s = rospy.ServiceProxy("return_joint_states", ReturnJointStates)
        resp = s(joint_names)
    except rospy.ServiceException, e:
        print "error when calling return_joint_states: %s"%e
        sys.exit(1)
    for (ind, joint_name) in enumerate(joint_names):
        if(not resp.found[ind]):
            print "joint %s not found!"%joint_name
    
    """
    point_cloud_2 = None

    try:
	print "Before service call"
        s = rospy.ServiceProxy("return_point_cloud", ReturnPointCloud)
        point_cloud_2 = s().point_cloud
	print "Successful point cloud!!!!!!"
    except rospy.ServiceException, e:
        print "error when calling return_point_cloud: %s"%e
        sys.exit(1)
    """

    #for (ind, joint_name) in enumerate(joint_names):
    #    if(not resp.found[ind]):
    #        print "joint %s not found!"%joint_name


    #scene_dir = osp.join(trajoptpy.bigdata_dir, args.scene_name)
    #if not args.dry_run: os.mkdir(scene_dir)


    listener = tf.TransformListener()
    listener.waitForTransform("/base_footprint","/head_mount_kinect_rgb_optical_frame", rospy.Time(0),rospy.Duration(1))
    (x,y,z), (qx,qy,qz,qw) = listener.lookupTransform("/base_footprint","/head_mount_kinect_rgb_optical_frame", rospy.Time(0))
    T_w_k = openravepy.matrixFromPose([qw,qx,qy,qz,x,y,z])
    #if not args.dry_run: np.savetxt(osp.join(scene_dir, "kinect_frame.txt"), T_w_k)

    dof_vals = robot.GetDOFValues()
    #if not args.dry_run: np.savetxt(osp.join(scene_dir,"dof_vals.txt"), dof_vals)


    grabber=cloudprocpy.CloudGrabber()


    xyzrgb = None
    xyzrgb = grabber.getXYZRGB()
    #if not args.dry_run: xyzrgb.save(osp.join(scene_dir,"cloud.pcd"))

    
    #time.sleep(5)

    print "Before making mesh"
    
    if xyzrgb != None:
    	print "Point cloud from grabber"
        xyzrgb.save("cloud.pcd")
    point_cloud = cloudprocpy.readPCDXYZ("cloud.pcd")
    """
    elif (point_cloud == None) and (point_cloud_2 == None):
        print "No point cloud"
    elif point_cloud_2 != None:
        point_cloud = point_cloud_2
    """
    #print "Point Cloud ROS: "
    #print point_cloud_2

    print "Cloud Grabber: "
    print xyzrgb

    #cloud_orig = cloudprocpy.readPCDXYZ("cloud.pcd") 
    mesh = generate_mesh(point_cloud)
    mesh_body = mk.create_trimesh(env, get_xyz_world_frame(mesh.getCloud()), np.array(mesh.getFaces()), name="simple_mesh")
    mesh_body.SetUserData("bt_use_trimesh", True) # Tell collision checker to use the trimesh rather than the convex hull of it
    print "After making mesh"

    
    joint_start = resp.position

    print "Joint start: " + str(joint_start)
    robot.SetDOFValues(joint_start) # robot.GetManipulator('rightarm').GetArmIndices())
    time.sleep(3)

    roll = 0.0
    pitch = 0.0
    yaw = 0
    quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    quat_target = [quat[3], quat[0], quat[1], quat[2]] # wxyz
    xyz_target = [0.5, -0.2, 0.5]
    hmat_target = openravepy.matrixFromPose( np.r_[quat_target, xyz_target] )

    # BEGIN ik
    init_joint_target = None
    manip = robot.GetManipulator("rightarm")
    init_joint_target = ku.ik_for_link(hmat_target, manip, "r_gripper_tool_frame",
            filter_options = openravepy.IkFilterOptions.CheckEnvCollisions)

    tries = 0
    while ((init_joint_target == None) and (tries < 100)):
        tries = tries + 1
        xyz_target[0] = xyz_target[0] + random.uniform(-0.05, 0.05)
        xyz_target[1] = xyz_target[1] + random.uniform(-0.05, 0.05)
        xyz_target[2] = xyz_target[2] + random.uniform(-0.05, 0.05)
        init_joint_target = ku.ik_for_link(hmat_target, manip, "r_gripper_tool_frame",
            filter_options = openravepy.IkFilterOptions.CheckEnvCollisions)

    print "Final target: " + str(xyz_target) + " Took " + str(tries) + " tries."
    # END ik
    print "Target: " + str(init_joint_target)
    robot.SetDOFValues(init_joint_target, robot.GetManipulator('rightarm').GetArmIndices())
    time.sleep(3)

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
    prob = trajoptpy.ConstructProblem(s, env) # create object that stores optimization problem
    result = trajoptpy.OptimizeProblem(prob) # do optimization
    print result

    from trajoptpy.check_traj import traj_is_safe
    prob.SetRobotActiveDOFs() # set robot DOFs to DOFs in optimization problem
    #assert traj_is_safe(result.GetTraj(), robot) # Check that trajectory is collision free

    # Replay the trajectory on OpenRAVE
    time.sleep(2)
    for position in result.GetTraj():
        robot.SetDOFValues(position, robot.GetManipulator('rightarm').GetArmIndices())
        time.sleep(1)

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
    if sum_costs < 20:
        arm = Arm('r_arm')
        arm.move(result.GetTraj())
    else:
        print "Cost of " + str(sum_costs) + " is too big."

