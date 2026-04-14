# wheel_stop_to_goal_node.py

import copy

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped


class WheelStopToGoalNode(Node):
    def __init__(self):
        super().__init__('wheel_stop_to_goal_node')

        self.latest_pose = None
        self.goal_sent = False
      
        self.pose_sub = self.create_subscription(
            PoseStamped, '/aruco_pose_base', self.pose_callback, 10
        )
        self.status_sub = self.create_subscription(
            String, '/wheel_status', self.status_callback, 10
        )
        self.goal_pub = self.create_publisher(
            PoseStamped, '/mirobot_goal_pose', 10
        )
      
        # Target offset, Based on target (TBD)
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.offset_z = 0.0

    def pose_callback(self, msg):
        self.latest_pose = msg

    # Check wheel status
    def status_callback(self, msg):
        if msg.data.strip().lower() != 'stopped':
            return

        if self.goal_sent:
            return

        if self.latest_pose is None:
            self.get_logger().warn('No pose received yet.')
            return

        goal_pose = copy.deepcopy(self.latest_pose)
        goal_pose.pose.position.x += self.offset_x
        goal_pose.pose.position.y += self.offset_y
        goal_pose.pose.position.z += self.offset_z

        # send goal pose
        self.goal_pub.publish(goal_pose)
        self.goal_sent = True
        self.get_logger().info('Published /mirobot_goal_pose')

def main(args=None):
    rclpy.init(args=args)
    node = WheelStopToGoalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
