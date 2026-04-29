import copy
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class WheelStopToGoalNode(Node):
    def __init__(self):
        super().__init__("wheel_stop_to_goal_node")

        defaults = {
            "pose_topic": "/aruco_pose_base",
            "wheel_status_topic": "/wheel_status",
            "goal_topic": "/mirobot_goal_pose",
            "sample_delay_sec": 0.2,
            "sample_count": 5,
            "offset_x": 0.0,
            "offset_y": 0.0,
            "offset_z": 0.0,
            "offset_frame": "marker",
            "goal_frame": "base_link",
            "use_marker_orientation": True,
            "goal_qx": 0.0,
            "goal_qy": 0.0,
            "goal_qz": 0.0,
            "goal_qw": 1.0,
        }

        # parameter declare
        for name, value in defaults.items():
            self.declare_parameter(name, value)
            setattr(self, name, self.get_parameter(name).value)

        # initialize
        self.latest_pose = None
        self.prev_is_stopped = False
        self.waiting_for_first_pose_after_stop = False
        self.collecting = False
        self.sample_buffer = []
        self.delay_timer = None

        
        # make subscriber
        self.pose_sub = self.create_subscription(
            PoseStamped, self.pose_topic, self.pose_callback, 10
        )
        self.status_sub = self.create_subscription(
            String, self.wheel_status_topic, self.status_callback, 10
        )
        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)

    def reset_sampling(self):
        self.collecting = False
        self.waiting_for_first_pose_after_stop = False
        self.sample_buffer = []
        if self.delay_timer is not None:
            self.delay_timer.cancel()
            self.delay_timer = None

    def pose_callback(self, msg):
        self.latest_pose = msg

        if self.waiting_for_first_pose_after_stop:
            self.waiting_for_first_pose_after_stop = False
            self.schedule_sampling()
            return

        if not self.collecting:
            return

        self.sample_buffer.append(copy.deepcopy(msg))
        if len(self.sample_buffer) >= self.sample_count:
            self.publish_averaged_goal()
            self.collecting = False
            self.sample_buffer = []

    def status_callback(self, msg):
        is_stopped = msg.data.strip().lower() == "stopped"

        if not is_stopped:
            self.prev_is_stopped = False
            self.reset_sampling()
            return

        if self.prev_is_stopped:
            return

        if self.latest_pose is None:
            self.prev_is_stopped = True
            self.waiting_for_first_pose_after_stop = True
            self.get_logger().warn(
                "Wheel stopped before any pose was received. "
                "Sampling will start when the first pose arrives."
            )
            return

        self.prev_is_stopped = True
        self.schedule_sampling()

    def schedule_sampling(self):
        self.reset_sampling()
        self.delay_timer = self.create_timer(self.sample_delay_sec, self.start_sampling_once)

        self.get_logger().info(
            "Wheel stopped. Waiting %.3f sec before collecting %d samples."
            % (self.sample_delay_sec, self.sample_count)
        )

    def start_sampling_once(self):
        self.reset_sampling()
        self.collecting = True
        self.get_logger().info("Started pose sampling.")

    def publish_averaged_goal(self):
        if not self.sample_buffer:
            self.get_logger().warn("No samples collected.")
            return

        avg_x = sum(p.pose.position.x for p in self.sample_buffer) / len(self.sample_buffer)
        avg_y = sum(p.pose.position.y for p in self.sample_buffer) / len(self.sample_buffer)
        avg_z = sum(p.pose.position.z for p in self.sample_buffer) / len(self.sample_buffer)

        goal_pose = copy.deepcopy(self.sample_buffer[-1])
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = self.goal_frame

        offset_x, offset_y, offset_z = self._resolve_offset_in_goal_frame(goal_pose)
        goal_pose.pose.position.x = avg_x + offset_x
        goal_pose.pose.position.y = avg_y + offset_y
        goal_pose.pose.position.z = avg_z + offset_z

        if not self.use_marker_orientation:
            goal_pose.pose.orientation.x = self.goal_qx
            goal_pose.pose.orientation.y = self.goal_qy
            goal_pose.pose.orientation.z = self.goal_qz
            goal_pose.pose.orientation.w = self.goal_qw

        self.goal_pub.publish(goal_pose)
        self.get_logger().info(
            "Published averaged goal: x=%.4f y=%.4f z=%.4f"
            % (
                goal_pose.pose.position.x,
                goal_pose.pose.position.y,
                goal_pose.pose.position.z,
            )
        )

    def _resolve_offset_in_goal_frame(self, marker_pose):
        offset = (float(self.offset_x), float(self.offset_y), float(self.offset_z))

        if str(self.offset_frame).lower() in ("base", "goal", "base_link"):
            return offset

        q = marker_pose.pose.orientation
        qx, qy, qz, qw = self._normalize_quaternion(q.x, q.y, q.z, q.w)
        return self._rotate_vector_by_quaternion(offset, (qx, qy, qz, qw))

    @staticmethod
    def _normalize_quaternion(x, y, z, w):
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm < 1e-12:
            raise ValueError("Marker pose has invalid zero quaternion.")
        return x / norm, y / norm, z / norm, w / norm

    @staticmethod
    def _rotate_vector_by_quaternion(vector, quaternion):
        vx, vy, vz = vector
        qx, qy, qz, qw = quaternion

        # Efficient form of q * v * q^-1 for a unit quaternion.
        tx = 2.0 * (qy * vz - qz * vy)
        ty = 2.0 * (qz * vx - qx * vz)
        tz = 2.0 * (qx * vy - qy * vx)

        rx = vx + qw * tx + (qy * tz - qz * ty)
        ry = vy + qw * ty + (qz * tx - qx * tz)
        rz = vz + qw * tz + (qx * ty - qy * tx)
        return rx, ry, rz


def main(args=None):
    rclpy.init(args=args)
    node = WheelStopToGoalNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
