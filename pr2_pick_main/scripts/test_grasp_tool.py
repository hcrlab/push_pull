#! /usr/bin/env python

from states.GraspTool import GraspTool

# To execute GraspTool state while developing it

class MockUserdata(object):
    ''' Stand-in for smach's userdata object '''


def main():
    state = GraspTool()
    mock_user_data = MockUserdata()
    mock_user_data.debug = True
    result = state.execute(mock_user_data)
    print result


if __name__ == '__main__':
    main()
