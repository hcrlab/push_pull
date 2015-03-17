import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--interactive", action="store_true")
parser.add_argument("--position_only", action="store_true")
args = parser.parse_args()

import openravepy
import trajoptpy
import json
import numpy as np
import trajoptpy.kin_utils as ku
from joint_states_listener.srv import ReturnJointStates
import time
import sys
from sensor_msgs.msg import JointState
import threading
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import actionlib
import rospy

class Arm:
    def __init__(self, arm_name):
        #arm_name should be l_arm or r_arm
        self.name = arm_name
        self.jta = actionlib.SimpleActionClient('/'+arm_name+'_controller/joint_trajectory_action',
                        JointTrajectoryAction)
        rospy.loginfo('Waiting for joint trajectory action')
        self.jta.wait_for_server()
        rospy.loginfo('Found joint trajectory action!')

    def move(self, angles):
        goal = JointTrajectoryGoal()
        char = self.name[0] #either 'r' or 'l'
        goal.trajectory.joint_names = [char+'_shoulder_pan_joint',
                   char+'_shoulder_lift_joint',
                   char+'_upper_arm_roll_joint',
                   char+'_elbow_flex_joint',
                   char+'_forearm_roll_joint',
                   char+'_wrist_flex_joint',
                   char+'_wrist_roll_joint']
        point = JointTrajectoryPoint()
        point.positions = angles
        point.time_from_start = rospy.Duration(3)
        goal.trajectory.points.append(point)
        self.jta.send_goal_and_wait(goal)


 
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

    
    joint_start = resp.position

    print "Joint start: " + str(joint_start)
    robot.SetDOFValues(joint_start) # robot.GetManipulator('rightarm').GetArmIndices())
    time.sleep(3)

    quat_target = [1,0,0,0] # wxyz
    xyz_target = [0.55,  0.20, 0.8]
    hmat_target = openravepy.matrixFromPose( np.r_[quat_target, xyz_target] )

    # BEGIN ik
    manip = robot.GetManipulator("rightarm")
    init_joint_target = ku.ik_for_link(hmat_target, manip, "r_gripper_tool_frame",
        filter_options = openravepy.IkFilterOptions.CheckEnvCollisions)
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
    assert traj_is_safe(result.GetTraj(), robot) # Check that trajectory is collision free

    # Replay the trajectory on OpenRAVE
    time.sleep(10)
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

    arm = Arm('r_arm')
    for position in result.GetTraj():
        arm.move(position)


