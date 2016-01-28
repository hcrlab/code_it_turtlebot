#!/usr/bin/env python

from std_msgs.msg import Empty
from code_it.srv import DisplayMessage
import display
import rospy

def onProgramEnd(msg):
    display.display_default()

def main():
    rospy.Service('code_it/api/display_message', DisplayMessage, display.on_display_message) 
    rospy.Subscriber("code_it/stopped", Empty, onProgramEnd)

if __name__ == '__main__':
    rospy.init_node('code_it_turtlebot')
    main()
    rospy.spin()
