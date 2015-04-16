#!/usr/bin/env python  
from geometry_msgs.msg import TransformStamped
from pr2_pick_perception.srv import DeleteStaticTransform, DeleteStaticTransformRequest, DeleteStaticTransformResponse
from pr2_pick_perception.srv import SetStaticTransform, SetStaticTransformRequest, SetStaticTransformResponse
import rospy
import tf
import tf2_ros


class StaticTransformManager(object):
    def __init__(self):
        self._transforms = {} # A dictionary mapping (parent, child) to static transforms.
        self._set_service = rospy.Service('perception/set_static_transform', SetStaticTransform, self.set_transform)
        self._delete_service = rospy.Service('perception/delete_static_transform', DeleteStaticTransform, self.delete_transform)
        self._broadcaster = tf2_ros.TransformBroadcaster()

    def set_transform(self, req):
        parent_frame = rospy.resolve_name(req.transform.header.frame_id)
        child_frame = rospy.resolve_name(req.transform.child_frame_id)
        self._transforms[(parent_frame, child_frame)] = req.transform
        return SetStaticTransformResponse(True)

    def delete_transform(self, req):
        req = DeleteStaticTransformRequest()
        parent_frame = rospy.resolve_name(req.parent_frame)
        child_frame = rospy.resolve_name(req.child_frame)
        if (parent_frame, child_frame) in self._transforms:
            del self._transforms[(parent_frame, child_frame)]
        return DeleteStaticTransformResponse(True)

    def broadcast_all(self):
        for transform in self._transforms.values():
            transform.header.stamp = rospy.Time.now()
            self._broadcaster.sendTransform(transform)


def main():
    rospy.init_node('static_tf')
    m = StaticTransformManager()
    def broadcast_all(event):
        m.broadcast_all()
    rospy.Timer(rospy.Duration(0.01), broadcast_all)
    rospy.spin()

if __name__ == '__main__':
    main()
