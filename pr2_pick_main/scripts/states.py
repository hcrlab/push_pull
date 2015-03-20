import outcomes
import rospy
import smach


class StartPose(smach.State):
    """Sets the robot's starting pose at the beginning of the challenge.
    """
    name = 'START_POSE'

    def __init__(self):
        smach.State.__init__(self,
            outcomes=[
                outcomes.START_POSE_SUCCESS,
                outcomes.START_POSE_FAILURE
            ]
        )

    def execute(self, userdata):
        rospy.loginfo('Setting start pose.')
        return outcomes.START_POSE_SUCCESS


class FindShelf(smach.State):
    """Localizes the shelf.
    """
    name = 'FIND_SHELF'

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=[
                outcomes.FIND_SHELF_SUCCESS,
                outcomes.FIND_SHELF_FAILURE
            ]
        )

    def execute(self, userdata):
        rospy.loginfo('Finding shelf.')
        return outcomes.FIND_SHELF_SUCCESS


class UpdatePlan(smach.State):
    """Decides on the next object to pick.
    """
    name = 'UPDATE_PLAN'

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=[
                outcomes.UPDATE_PLAN_NEXT_OBJECT,
                outcomes.UPDATE_PLAN_NO_MORE_OBJECTS,
                outcomes.UPDATE_PLAN_FAILURE
            ],
            input_keys=['last_bin_id'],
            output_keys=['last_bin_id', 'bin_id']
        )

    def execute(self, userdata):
        rospy.loginfo('Updating plan.')
        if userdata.last_bin_id is None:
            userdata.last_bin_id = -1

        if userdata.last_bin_id == 2:
            return outcomes.UPDATE_PLAN_NO_MORE_OBJECTS
        elif userdata.last_bin_id >= -1:
            bin_id = userdata.last_bin_id + 1
            userdata.bin_id = bin_id
            userdata.last_bin_id = bin_id
            return outcomes.UPDATE_PLAN_NEXT_OBJECT
        else:
            return outcomes.UPDATE_PLAN_FAILURE


class PrepareSensing(smach.State):
    """Sets the robot's pose before sensing a bin.
    """
    name = 'PREPARE_SENSING'

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=[
                outcomes.PREPARE_SENSING_SUCCESS,
                outcomes.PREPARE_SENSING_FAILURE
            ],
            input_keys=['bin_id']
        )

    def execute(self, userdata):
        rospy.loginfo('Preparing to sense bin {}'.format(userdata.bin_id))
        return outcomes.PREPARE_SENSING_SUCCESS


class SenseBin(smach.State):
    """Performs sensing on a bin.
    """
    name = 'SENSE_BIN'

    def __init__(self):

        smach.State.__init__(
            self,
            outcomes=[
                outcomes.SENSE_BIN_SUCCESS,
                outcomes.SENSE_BIN_NO_OBJECTS,
                outcomes.SENSE_BIN_FAILURE
            ],
            input_keys=['bin_id']
        )

    def execute(self, userdata):
        rospy.loginfo('Sensing bin {}'.format(userdata.bin_id))
        return outcomes.SENSE_BIN_SUCCESS


class PrepareManipulation(smach.State):
    """Sets the robot's pose before manipulating an object or objects in a bin.
    """
    name = 'PREPARE_MANIPULATION'

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=[
                outcomes.PREPARE_MANIPULATION_SUCCESS,
                outcomes.PREPARE_MANIPULATION_FAILURE
            ],
            input_keys=['bin_id']
        )

    def execute(self, userdata):
        rospy.loginfo('Preparing manipulation for bin {}'.format(userdata.bin_id))
        return outcomes.PREPARE_MANIPULATION_SUCCESS


class ManipulateObject(smach.State):
    """Manipulates the target object (grasp, scoop, etc.)
    """
    name = 'MANIPULATE_OBJECT'

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=[
                outcomes.MANIPULATE_OBJECT_SUCCESS,
                outcomes.MANIPULATE_OBJECT_FAILURE
            ],
            input_keys=['bin_id']
        )

    def execute(self, userdata):
        rospy.loginfo('Manipulating object in bin {}'.format(userdata.bin_id))
        return outcomes.MANIPULATE_OBJECT_SUCCESS


class DropOffObject(smach.State):
    """Deposits the object or objects into the order bin.
    """
    name = 'DROP_OFF_OBJECT'

    def __init__(self):
        smach.State.__init__(self,
            outcomes=[
                outcomes.DROP_OFF_OBJECT_SUCCESS,
                outcomes.DROP_OFF_OBJECT_FAILURE
            ],
            input_keys=['bin_id']
        )

    def execute(self, userdata):
        rospy.loginfo('Dropping off object from bin {}'.format(userdata.bin_id))
        return outcomes.DROP_OFF_OBJECT_SUCCESS
