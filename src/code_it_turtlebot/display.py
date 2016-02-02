# Implements a display using the Blinky face.

import blinky.msg
import rospy


class Display(object):
    def __init__(self, blinky_client):
        """Create a Display object.

        Args:
            blinky_client: actionlib.SimpleActionClient for Blinky.
        """
        self._blinky_client = blinky_client

    def show_default(self):
        self._blinky_client.wait_for_server()
        goal = blinky.msg.FaceGoal()
        goal.display_type = 'default'
        self._blinky_client.send_goal(goal)
        self._blinky_client.wait_for_result()

    def show_message(self, h1_text, h2_text):
        self._blinky_client.wait_for_server()
        goal = blinky.msg.FaceGoal()
        goal.display_type = 'displayMessage'
        goal.h1_text = h1_text
        goal.h2_text = h2_text
        self._blinky_client.send_goal(goal)
        self._blinky_client.wait_for_result()

    def ask_multiple_choice(self, question, choices):
        self._blinky_client.wait_for_server()
        goal = blinky.msg.FaceGoal()
        goal.display_type = 'askMultipleChoice'
        goal.question = question
        goal.choices = choices
        self._blinky_client.send_goal(goal)
        self._blinky_client.wait_for_result()
        return self._blinky_client.get_result()
