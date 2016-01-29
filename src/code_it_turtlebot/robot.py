from display import Display
from nav import Navigation
import actionlib
import blinky.msg
from kobuki_msgs.msg import AutoDockingAction
from move_base_msgs.msg import MoveBaseAction
import tf


class Robot(object):
    def __init__(self, display, navigation):
        self.display = display
        self.navigation = navigation


def build_real():
    blinky_client = actionlib.SimpleActionClient('blinky',
                                                 blinky.msg.FaceAction)
    display = Display(blinky_client)

    move_base_client = actionlib.SimpleActionClient('move_base',
                                                    MoveBaseAction)
    dock_client = actionlib.SimpleActionClient('dock_drive_action',
                                                    AutoDockingAction)
    tf_listener = tf.TransformListener()
    navigation = Navigation('base_footprint', 'map', tf_listener,
                            move_base_client, dock_client)

    robot = Robot(display, navigation)
    return robot
