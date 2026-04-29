import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
import serial
import math
import time

from wlkatapython import Mirobot_UART

class TrajectorySubscriber(Node):
    
    def __init__(self):
        super().__init__('trajectory_subscriber')

        self.declare_parameter("trajectory_topic", "/mirobot_joint_trajectory")
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("serial_timeout", 1.0)
        self.declare_parameter("do_homing", True)
        self.declare_parameter("gcode_motion", "G01")
        self.declare_parameter("min_point_delay_sec", 0.02)

        self.trajectory_topic = str(self.get_parameter("trajectory_topic").value)
        self.serial_port = str(self.get_parameter("serial_port").value)
        self.baud_rate = int(self.get_parameter("baud_rate").value)
        self.serial_timeout = float(self.get_parameter("serial_timeout").value)
        self.do_homing = bool(self.get_parameter("do_homing").value)
        self.gcode_motion = str(self.get_parameter("gcode_motion").value)
        self.min_point_delay_sec = float(
            self.get_parameter("min_point_delay_sec").value
        )

        try:
            self.ser = serial.Serial(
                self.serial_port,
                self.baud_rate,
                timeout=self.serial_timeout,
            )
            self.get_logger().info("Serial port opened successfully.")
            self.mirobot=Mirobot_UART(True,False)
            self.mirobot.init(self.ser,-1)
            if self.do_homing:
                self.get_logger().info("arm homing...")
                self.mirobot.homing()
            self.get_logger().info("Ok,Waiting for motion command of robotic arm")

        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise

        self.subscription = self.create_subscription(
            JointTrajectory,
            self.trajectory_topic,
            self.listen_trajectory,
            10
        )

    def listen_trajectory(self, msg):
        if not msg.points:
            self.get_logger().warn("Trajectory has no points.")
            return

        self.get_logger().info(f"Processing {len(msg.points)} trajectory points.")
        previous_time = 0.0

        for point in msg.points:
            if len(point.positions) < 6:
                self.get_logger().warn("Skipping point with fewer than 6 joints.")
                continue

            # Convert radians to degrees
            degs = [round((p / math.pi) * 180, 2) for p in point.positions[:6]]
            j1, j2, j3, j4, j5, j6 = degs

            # Format G-code with spaces and newline
            command = (
                f"M21 G90 {self.gcode_motion} "
                f"X{j1} Y{j2} Z{j3} A{j4} B{j5} C{j6}\n"
            )
            self.get_logger().debug(f"Sending: {command.strip()}")

            try:
                self.ser.write(command.encode('utf-8'))
                response = self.ser.readline()
                if response:
                    self.get_logger().info(f"Response: {response.decode().strip()}")
                else:
                    self.get_logger().warn("No response from device (timeout).")
            except Exception as e:
                self.get_logger().error(f"Serial communication error: {e}")
                break  # or continue, depending on robustness needs

            current_time = self._duration_to_sec(point.time_from_start)
            delay = max(current_time - previous_time, self.min_point_delay_sec)
            previous_time = current_time
            time.sleep(delay)

        self.get_logger().info("Finished sending trajectory segment.")
        print("___________________________________")

    @staticmethod
    def _duration_to_sec(duration):
        return float(duration.sec) + float(duration.nanosec) * 1e-9

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Serial port closed.")
        super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = TrajectorySubscriber()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
