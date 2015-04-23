import rospy
import smach

import outcomes


class GraspTool(smach.State):
    ''' Grasps tool from the robot's right shoulder holster '''
    name = 'GRASP_TOOL'

    def __init__(self, **services):
        smach.State.__init__(
            self,
            outcomes=[
                outcomes.GRASP_TOOL_SUCCESS,
                outcomes.GRASP_TOOL_FAILURE,
            ],
            input_keys=['debug'],
        )

    def execute(self, userdata):
        return outcomes.GRASP_TOOL_SUCCESS
