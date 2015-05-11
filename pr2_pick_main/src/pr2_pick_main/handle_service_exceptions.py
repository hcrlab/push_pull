import rospy


def handle_service_exceptions(outcome):
    """A decorator for smach states to handle all service exceptions.
    Makes the state return the given outcome on failure.

    Usage:
    @handle_service_exceptions(outcomes.FIND_SHELF_FAILURE)
    def execute(self, userdata):
        ...
    """

    def with_outcome(execute):
        def wrapped_execute(self, userdata):
            try:
                return execute(self, userdata)
            except rospy.ServiceException as e:
                rospy.logerr(e)
                return outcome

        return wrapped_execute

    return with_outcome
