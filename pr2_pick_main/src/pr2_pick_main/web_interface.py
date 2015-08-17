from pr2_pick_main.msg import WebInterfaceParams
from pr2_pick_main.msg import WebInterfaceSubmission
import json
import random
import rospy
import time


class WebInterface(object):
    def __init__(self):
        self._interface_publisher = rospy.Publisher('pr2_pick_main/web_interface/interface_params', WebInterfaceParams)

    def _publish_params(self, msg):
        rate = rospy.Rate(5)
        while self._interface_publisher.get_num_connections() == 0:
            rate.sleep() 
        self._interface_publisher.publish(msg)

    def display_default(self):
        """Displays the default screen.
        """
        msg = WebInterfaceParams()
        msg.interface_name = 'generic_interface'
        msg.interface_type = 'default'
        self._publish_params(msg)

    def ask_choice(self, message, choices, timeout=None, has_countdown=False):
        """Asks the user a multiple choice question.

        Displays a button for each choice in choices. This method blocks until
        the given timeout. If the timeout is exceeded, then None is returned.
        If no timeout is supplied, then this method blocks until a response is
        given.

        Args:
          message: string. The question to ask.
          choices: list of strings. The choices to offer to the user.
          timeout: float. The time, in seconds, to wait for a response, or None
            to wait forever.

        Returns: string. The choice that was selected.
        """
        msg = WebInterfaceParams()
        msg.interface_type = 'ask_choice'
        msg.interface_name = 'generic_interface'
        prompt_id = str(random.randint(0, 1000000))
        msg.keys = ['message', 'choices', 'prompt_id']
        msg.values = [message, json.dumps(choices), prompt_id]
        self._publish_params(msg)

        # A workaround for the fact that you can't provide services in roslibjs.
        # Normally, we would call a service to set the interface and get the user
        # response, but we can only communicate over topics. We include a random ID
        # with the request and response messages and check if the IDs match.
        submission = None
        response_prompt_id = None  # The prompt_id returned in the response.
        timeout_remaining = timeout
        choice = None
        while response_prompt_id != prompt_id:
            start_time = rospy.Time().now()
            submission = rospy.wait_for_message(
                'pr2_pick_main/web_interface/interface_submission',
                WebInterfaceSubmission, timeout_remaining)

            # If a timeout is set, then possibly break out of the loop.
            wait_duration = (rospy.Time().now() - start_time).to_sec()
            if timeout is not None:
                timeout_remaining = timeout - wait_duration
                if timeout_remaining <= 0:
                    break

            if submission is None:
                continue
            if submission.interface_type != 'ask_choice':
                continue
            if len(submission.keys) != len(submission.values):
                rospy.logerr('[Interface]: unequal keys and values.')
                break
            params = {}
            for k, v in zip(submission.keys, submission.values):
                params[k] = v
            if 'prompt_id' not in params:
                rospy.logerr('[Interface]: no prompt_id given.')
                break
            response_prompt_id = params['prompt_id']
            if response_prompt_id != prompt_id:
                continue
            if 'choice' not in params:
                rospy.logerr('[Interface]: no choice given.')
                break

            choice = params['choice']
        self.display_default()
        return choice


    def get_floats(self, message, param_names, param_mins, param_maxs, param_values):
        msg = WebInterfaceParams()
        msg.interface_type = 'get_floats'
        msg.interface_name = 'generic_interface'
        prompt_id = str(random.randint(0, 1000000))

        sliders = []
        for i in range(len(param_names)):
            sliders = (sliders + [{'slider_name':param_names[i],
                                    'slider_min':param_mins[i],
                                    'slider_max':param_maxs[i],
                                    'slider_step':0.01,
                                    'slider_values':param_values[i]}])
        print 'sliders', sliders

        msg.keys = ['message', 'sliders', 'prompt_id']
        msg.values = [message, json.dumps(sliders), prompt_id]
        self._publish_params(msg)

        # A workaround for the fact that you can't provide services in roslibjs.
        # Normally, we would call a service to set the interface and get the user
        # response, but we can only communicate over topics. We include a random ID
        # with the request and response messages and check if the IDs match.
        submission = None
        response_prompt_id = None  # The prompt_id returned in the response.
        timeout_remaining = None
        choice = None
        while response_prompt_id != prompt_id:
            start_time = rospy.Time().now()
            submission = rospy.wait_for_message(
                'pr2_pick_main/web_interface/interface_submission',
                WebInterfaceSubmission, timeout_remaining)

            # If a timeout is set, then possibly break out of the loop.
            wait_duration = (rospy.Time().now() - start_time).to_sec()
            if timeout is not None:
                timeout_remaining = timeout - wait_duration
                if timeout_remaining <= 0:
                    break

            if submission is None:
                continue
            if submission.interface_type != 'get_floats':
                continue
            if len(submission.keys) != len(submission.values):
                rospy.logerr('[Interface]: unequal keys and values.')
                break
            params = {}
            for k, v in zip(submission.keys, submission.values):
                params[k] = v
            if 'prompt_id' not in params:
                rospy.logerr('[Interface]: no prompt_id given.')
                break
            response_prompt_id = params['prompt_id']
            if response_prompt_id != prompt_id:
                continue
            if 'values' not in params:
                rospy.logerr('[Interface]: no choice given.')
                break

            values = json.loads(params['values'])

        self.display_default()
        return values

    def display_message(self, message, duration=None, has_countdown=False):
        """Displays the given message on the screen.

        If no duration is given, then the message is shown and this method
        returns immediately. Otherwise, this method blocks for the given
        duration while the message is being shown.

        Args:
          message: string. The message to show.
          duration: float. The time, in seconds, to show the message, or None to 
            show the message indefinitely.
        """
        msg = WebInterfaceParams()
        msg.interface_type = 'display_message'
        msg.interface_name = 'generic_interface'
        msg.keys = ['message']

        if has_countdown:
            if duration is not None:
                start_time = rospy.Time().now()
                time_remaining = duration
                displayed_time_remaining = None

                while (time_remaining > 0):

                    if displayed_time_remaining != int(time_remaining):
                        displayed_time_remaining = int(time_remaining)
                        displayed_message = (message + '\n Time remaing:' +
                            str(displayed_time_remaining) + ' seconds')
                        msg.values = [displayed_message]
                        self._publish_params(msg)
                    
                    time.sleep(0.05)
                    time_remaining = duration - (rospy.Time().now() - start_time).to_sec()
                self.display_default()
            else:
                rospy.logerr('Cannot count down without a specified duration.')
        else:
            msg.values = [message]
            self._publish_params(msg)

            if duration is not None:
                    rospy.sleep(duration)
                    self.display_default()


