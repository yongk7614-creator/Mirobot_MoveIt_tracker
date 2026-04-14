# wheel_stop_to_goal_node.py

import copy

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

class WheelStopToGoalNode(Node):
  
