# Implements a display using the Blinky face.

import actionlib
import blinky.msg
import rospy
from code_it.srv import DisplayMessageResponse

# Display API
def display_default():
    client = actionlib.SimpleActionClient('blinky', blinky.msg.FaceAction) 
    client.wait_for_server()
    goal = blinky.msg.FaceGoal()
    goal.display_type = 'default'
    client.send_goal(goal)
    client.wait_for_result()


def display_message(h1_text, h2_text):
    client = actionlib.SimpleActionClient('blinky', blinky.msg.FaceAction) 
    client.wait_for_server()
    goal = blinky.msg.FaceGoal()
    goal.display_type = 'displayMessage'
    goal.h1_text = h1_text
    goal.h2_text = h2_text
    client.send_goal(goal)
    client.wait_for_result()

# ROS handlers
def on_display_message(request):
    display_message(request.h1_text, request.h2_text)
    return DisplayMessageResponse()
