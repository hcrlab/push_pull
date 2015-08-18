#!/usr/bin/env python

from pr2_pick_main.web_interface import WebInterface
import rospy

if __name__ == '__main__':

    rospy.init_node('pr2_pick_state_machine')

    _interface = WebInterface()
    values = [0.40, -0.05]

    while (True):
        new_values = _interface.get_floats(
            message="Choose parameters \r\n in list below",
            param_names=['param_a', 'param_b'],
            param_mins=[0.00, -0.10],
            param_maxs=[0.50, 0.10],
            param_values=values)
        print new_values
        values = new_values[:]
