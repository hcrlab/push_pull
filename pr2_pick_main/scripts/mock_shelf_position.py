#! /usr/bin/python

# Publishes a position relative to the odom frame, 20 times per second.
# Published position is the place where the robot would stand to be in
# front of the shelf and centered.
# This is meant to mock out the real shelf localization node for testing and
# while localization is still in development.

import rospy
import tf

if __name__ == "__main__":
    rospy.init_node('mock_shelf_position')
    broadcaster = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
        broadcaster.sendTransform(
            (2.3, -0.4, 0),
            tf.transformations.quaternion_from_euler(0, 0, 0),
            rospy.Time.now(),
            'shelf',
            'odom_combined'
        )
        rospy.sleep(rospy.Duration(0.05))
