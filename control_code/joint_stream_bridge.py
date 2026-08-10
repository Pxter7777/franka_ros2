import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import os
import socket
import sys
import threading
import time

class JointStreamBridgeNode(Node):
    """
    Bridges a non-ROS application to pxter_joint_stream_controller for STREAMED joint goals.

    Like bridge.py, but newline-framed so a dense stream of goals does not coalesce
    into one recv (the plain bridge.py reads one message per recv, which only works
    with slow senders like robot_commander.py). Each line is 7 comma-separated
    absolute joint positions in radians:
        q1,q2,q3,q4,q5,q6,q7
    Republished as Float64MultiArray on /pxter_joint_stream_controller/goal.
    """
    def __init__(self):
        super().__init__('pxter_joint_stream_bridge_node')

        self.publisher_ = self.create_publisher(Float64MultiArray, '/pxter_joint_stream_controller/goal', 10)
        self.get_logger().info('ROS 2 publisher on /pxter_joint_stream_controller/goal is ready.')

        self.host = 'localhost'
        # Required env — forwarded from the superproject's
        # configs/network(.defaults).conf; deliberately no local fallback.
        if "FRANKA_JOINT_STREAM_PORT" not in os.environ:
            sys.exit("FRANKA_JOINT_STREAM_PORT is not set — launch via the franka up-scripts, or pass docker exec -e FRANKA_JOINT_STREAM_PORT=<port>")
        self.port = int(os.environ["FRANKA_JOINT_STREAM_PORT"])

        self.server_thread = threading.Thread(target=self._socket_server_loop)
        self.server_thread.daemon = True
        self.server_thread.start()

    def _socket_server_loop(self):
        self.get_logger().info(f'Socket server starting on {self.host}:{self.port}')
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((self.host, self.port))
            s.listen()

            while rclpy.ok():
                try:
                    conn, addr = s.accept()
                    with conn:
                        self.get_logger().info(f'Socket connected by {addr}')
                        buffer = ''
                        while True:
                            data = conn.recv(1024)
                            if not data:
                                break  # Connection closed by client
                            buffer += data.decode('utf-8')
                            while '\n' in buffer:
                                line, buffer = buffer.split('\n', 1)
                                line = line.strip()
                                if line:
                                    self._handle_line(line)
                except Exception as e:
                    self.get_logger().error(f'Socket server error: {e}')
                    time.sleep(1)  # Avoid busy-looping on error

    def _handle_line(self, line):
        """ Parse one joint-goal line and publish it. """
        try:
            joints = [float(p) for p in line.split(',')]
        except ValueError as e:
            self.get_logger().error(f'Failed to parse joints "{line}": {e}')
            return

        if len(joints) != 7:
            self.get_logger().warn(
                f'Received invalid data: need 7 joints, got {len(joints)}')
            return

        msg = Float64MultiArray()
        msg.data = joints
        self.publisher_.publish(msg)
        self.get_logger().info(f'Relaying joint goal to ROS 2: {joints}')

def main(args=None):
    rclpy.init(args=args)
    bridge_node = JointStreamBridgeNode()
    try:
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        pass
    finally:
        bridge_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
