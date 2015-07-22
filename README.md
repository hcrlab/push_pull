## Push/Pull Manipulation for Graspability

### Introduction

Grasping is one of the most important and well-studied capabilities for robots 
with manipulators and grippers. It allows a robot to gain full control over an
object, enabling its transportation, reconfiguration, or modification. Despite the tremendous
progress in grasping research, driven by new perception and planning algorithms, grasping
objects in confined and cluttered environments is still difficult. 

One approach explored in the literature in dealing with such challenging environments is to use
non-prehensile manipulation of objects to reonfigure them in a way that
facilitates grasping. This can involve pushing the target object before grasping
it [dogar] or pushing other objects away to enable grasping of the target
object [dogar, others]. Previous work exploring this approach has focused on the
use of the robot's gripper or arm to accomplish the non-prehensile manipulation.
In this work, we propose using a simple tool specifically designed to manipulate
objects in confined environments. In addition to allowing push actions with point
and surface contacts, the high friction end-effector of the tool allows pull 
actions.

We present a method to apply the tool on non-greaspable objects so as to
make them graspable. Our method is based on predictive models of object-relative
tool actions. These models are represented with XXX and they are learned from experience. We
systematically evaluate our approach in the context of the Amazon Picking Challenge
tasks with a PR2. The task involves picking up target objects from a shelf with 
small cells (Xcm by Ycm). We demonstrate that the robot is able to use the tool
appropriately to grasp XX% of N situations in which M different objects are 