from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseGoal
import rospy


class Navigation(object):
    def __init__(self, base_frame, world_frame, tf_listener, move_base_client):
        """Create a Navigation object.
        
        Args:
            base_frame: string. The base frame of the robot.
            world_frame: string. The frame of the world.
            tf_listener: tf.TransformListener
            move_base_client: actionlib.SimpleActionClient for moving the base.
        """
        self._base_frame = base_frame
        self._world_frame = world_frame
        self._tf_listener = tf_listener
        self._move_base_client = move_base_client

    def go_to(self, pose_stamped):
        """Goes to a location in the world.

        Args:
          pose_stamped: geometry_msgs/PoseStamped. The location to go to.
        """
        self._move_base_client.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose = pose_stamped
        self._move_base_client.send_goal(goal)
        self._move_base_client.wait_for_result()

    def get_current_location(self):
        """Returns the location of the robot in the world frame.

        Returns: geometry_msgs/PoseStamped. The pose of the robot in the world
          frame, or None if it couldn't figure out its location.
        """
        try:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = self._base_frame
            pose_stamped.header.stamp = rospy.Time(0)
            pose_stamped.pose.position.x = 0
            pose_stamped.pose.position.y = 0
            pose_stamped.pose.position.z = 0
            pose_stamped.pose.orientation.w = 1
            pose_stamped.pose.orientation.x = 0
            pose_stamped.pose.orientation.y = 0
            pose_stamped.pose.orientation.z = 0
            current_location = self._tf_listener.transformPose(
                self._world_frame, pose_stamped)
            return current_location
        except Exception as e:
            rospy.logerr(
                'Failed to get current pose in world frame {}.'.format(
                    self._world_frame))
            rospy.logerr(e)
            return None
