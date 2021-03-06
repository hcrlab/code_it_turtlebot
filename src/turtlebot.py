#!/usr/bin/env python

from code_it_msgs.srv import AskMultipleChoice, AskMultipleChoiceResponse
from code_it_msgs.srv import DisplayMessage, DisplayMessageResponse
from code_it_msgs.srv import GoTo, GoToResponse
from code_it_msgs.srv import GoToDock, GoToDockResponse
from std_msgs.msg import Bool
import code_it_turtlebot as turtlebot
import location_db
import rospy


class RobotApi(object):
    """Implements the CodeIt! API, mainly through delegating to the robot object.
    """

    def __init__(self, robot, location_db):
        self._robot = robot
        self._location_db = location_db

    def on_display_message(self, request):
        self._robot.display.show_message(request.h1_text, request.h2_text)
        return DisplayMessageResponse()

    def on_ask_multiple_choice(self, request):
        result = self._robot.display.ask_multiple_choice(request.question,
                                                         request.choices)
        choice = result.choice if result is not None else None
        return AskMultipleChoiceResponse(choice=choice)

    def on_go_to(self, request):
        pose_stamped = self._location_db.get_by_name(request.location)
        if pose_stamped is None:
            msg = 'Location "{}" is not known to the robot.'.format(
                request.location)
            return GoToResponse(error=msg)
        result = self._robot.navigation.go_to(pose_stamped)
        return GoToResponse()

    def on_go_to_dock(self, request):
        pose_stamped = self._location_db.get_by_name('PreDockingPose')
        if pose_stamped is None:
            msg = 'PreDockingPose location not set.'.format(request.location)
            return GoToDockResponse(error=msg)
        result = self._robot.navigation.go_to(pose_stamped)
        result = self._robot.navigation.dock()
        return GoToDockResponse()

    def on_is_program_running(self, msg):
        if msg.data == False:
            self._robot.navigation.cancel()
            self._robot.display.show_default()


def main():
    robot = turtlebot.robot.build_real()
    db = location_db.build_real()
    api = RobotApi(robot, db)
    rospy.Service('code_it/api/display_message', DisplayMessage,
                  api.on_display_message)
    rospy.Service('code_it/api/ask_multiple_choice', AskMultipleChoice,
                  api.on_ask_multiple_choice)
    rospy.Service('code_it/api/go_to', GoTo, api.on_go_to)
    rospy.Service('code_it/api/go_to_dock', GoToDock, api.on_go_to_dock)
    rospy.Subscriber("code_it/is_program_running", Bool, api.on_is_program_running)


if __name__ == '__main__':
    rospy.init_node('code_it_turtlebot')
    main()
    rospy.spin()
