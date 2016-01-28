#!/usr/bin/env python

from code_it.srv import DisplayMessage, DisplayMessageResponse
from std_msgs.msg import Empty
import code_it_turtlebot as turtlebot
import rospy


class RobotApi(object):
    """Implements the CodeIt! API, mainly through delegating to the robot object.
    """

    def __init__(self, robot):
        self._robot = robot

    def on_display_message(request):
        robot.display.show_message(request.h1_text, request.h2_text)
        return DisplayMessageResponse()

    def on_program_end(msg):
        robot.display.show_default()


def main():
    robot = turtlebot.robot.build_real()
    api = RobotApi(robot)
    rospy.Service('code_it/api/display_message', DisplayMessage,
                  api.on_display_message)
    rospy.Subscriber("code_it/stopped", Empty, api.on_program_end)


if __name__ == '__main__':
    rospy.init_node('code_it_turtlebot')
    main()
    rospy.spin()
