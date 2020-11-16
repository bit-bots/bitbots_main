from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from humanoid_league_msgs.msg import HeadMode


class StartTimer(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(StartTimer, self).__init__(blackboard, dsd, parameters)

        if 'name' not in parameters:
            raise Exception('StartTimer: Name parameter is missing!')
        self.timer_name = parameters['name']
        if 'duration' not in parameters:
            raise Exception(f'StartTimer ({self.timer_name}): Duration parameter is missing!')
        self.duration = int(parameters['duration'])

    def perform(self, reevaluate=False):
        self.blackboard.blackboard.start_timer(self.timer_name, self.duration)
        return self.pop()


class EndTimer(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(EndTimer, self).__init__(blackboard, dsd, parameters)

        if 'name' not in parameters:
            raise Exception('EndTimer: Name parameter is missing!')
        self.timer_name = parameters['name']

    def perform(self, reevaluate=False):
        self.blackboard.blackboard.end_timer(self.timer_name)
        return self.pop()
