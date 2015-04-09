#!/usr/bin/env python

from pr2_pick_perception.srv import LocalizeShelf
from pr2_pick_perception.srv import LocalizeShelfResponse
import rospy

def handle_request(request):

    return LocalizeShelfResponse

def main():
    rospy.init_node('localize_shelf_service_node')
    rospy.Service('localize_shelf', LocalizeShelf, handle_request)
    rospy.spin()

if __name__ == '__main__':
    main()
