from display import Display
from nav import Navigation
import actionlib
import blinky.msg
import tf


class Robot(object):
    def __init__(self, display, navigation):
        self._display = display
        self._navigation = navigation


def build_real():
    blinky_client = actionlib.SimpleActionClient('blinky',
                                                 blinky.msg.FaceAction)
    display = Display(blinky_client)

    move_base_client = actionlib.SimpleActionClient('move_base',
                                                    MoveBaseAction)
    tf_listener = tf.TransformListener()
    navigation = Navigation('base_footprint', 'map', tf_listener,
                            move_base_client)

    robot = Robot(display, navigation)
    return robot
