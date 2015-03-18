#!/usr/bin/env python

'''
A ROS node providing the 'push_away' service, which enables the robot to
push objects in the positive x direction relative to its base.

It is indended for the case where there is not space to form a grip around
the target object because other objects are too close to it. The non-target
objects can be pushed away in order to make space to form a grip around the
target object.
'''

import copy
import sys

import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
import pr2_pick_manipulation
from pr2_pick_manipulation.srv import PushAway
import rospy

# Next:
# - handle failure gracefully
# - verification and error messages
# - return false if failed
# - test on real robot
# - remove tuck at end code

def push_away(request):
    rospy.loginfo('Pushing away from {}'.format(request.start))

    if request.left_hand:
        arm = 'left_arm'
    else:
        arm = 'right_arm'

    group = moveit_commander.MoveGroupCommander(arm)

    # TODO: verify that the goal state is valid

    # get position of push target from the request
    push_target = request.start
    print push_target

    # position 10 cm closer than the push target
    pre_push_position = copy.deepcopy(push_target)
    pre_push_position.position.x -= 0.1
    print pre_push_position

    # position 10 cm farther than push target
    post_push_position = copy.deepcopy(push_target)
    post_push_position.position.x += 0.1
    print post_push_position

    for pose in [pre_push_position, post_push_position, pre_push_position]:
        group.set_pose_target(pose)
        print 'Planning path to ', pose
        group.plan()
        print 'Going to pose now'
        group.go()

    # tuck the arm when done so we have something interesteing to do next time we run
    rest_pose = geometry_msgs.msg.Pose()
    rest_pose.orientation.x = 1.0
    rest_pose.position.x = 0.3
    rest_pose.position.y = 0.0
    rest_pose.position.z = 0.5
    group.set_pose_target(rest_pose)
    group.plan()
    group.go()

    return True


def test():
    target = geometry_msgs.msg.Pose()
    target.orientation.w = 1.0
    target.position.x = 0.5
    target.position.y = 0.1
    target.position.z = 1.1
    push_away_service = rospy.ServiceProxy('push_away', PushAway)
    result = push_away_service(target, True)


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('push_away', anonymous=True)
    rospy.Service('push_away', PushAway, push_away)

    # test()

    rospy.spin()
