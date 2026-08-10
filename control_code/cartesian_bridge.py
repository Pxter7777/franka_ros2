import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import os
import socket
import sys
import threading
import time

class CartesianBridgeNode(Node):
    """
    Bridges a non-ROS application to the pxter_cartesian_controller.

    Listens on a TCP socket for newline-delimited, comma-separated end-effector
    pose deltas and republishes them as Float64MultiArray on
    /pxter_cartesian_controller/goal. Each line is either:
        dx,dy,dz,droll,dpitch,dyaw            (6 values, pose delta only)
        dx,dy,dz,droll,dpitch,dyaw,gripper    (7 values; gripper is logged for now)
    Translation is in metres, rotation in radians (base frame).
    """
    def __init__(self):
        super().__init__('pxter_cartesian_bridge_node')

        self.publisher_ = self.create_publisher(
            Float64MultiArray, '/pxter_cartesian_controller/goal', 10)
        self.get_logger().info(
            'ROS 2 publisher on /pxter_cartesian_controller/goal is ready.')

        self.host = 'localhost'
        # Required env — forwarded from the superproject's
        # configs/network(.defaults).conf; deliberately no local fallback.
        if "FRANKA_CARTESIAN_BRIDGE_PORT" not in os.environ:
            sys.exit("FRANKA_CARTESIAN_BRIDGE_PORT is not set — launch via the franka up-scripts, or pass docker exec -e FRANKA_CARTESIAN_BRIDGE_PORT=<port>")
        self.port = int(os.environ["FRANKA_CARTESIAN_BRIDGE_PORT"])

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
                            # Process every complete (newline-terminated) line; keep
                            # any partial trailing line in the buffer for next recv.
                            while '\n' in buffer:
                                line, buffer = buffer.split('\n', 1)
                                line = line.strip()
                                if line:
                                    self._handle_line(line)
                except Exception as e:
                    self.get_logger().error(f'Socket server error: {e}')
                    time.sleep(1)  # Avoid busy-looping on error

    def _handle_line(self, line):
        """ Parse one delta line and publish it. """
        try:
            values = [float(p) for p in line.split(',')]
        except ValueError as e:
            self.get_logger().error(f'Failed to parse delta "{line}": {e}')
            return

        if len(values) not in (6, 7):
            self.get_logger().warn(
                f'Received invalid data: need 6 or 7 values, got {len(values)}')
            return

        if len(values) == 7:
            # Gripper command is not wired to franka_gripper yet; log and drop it.
            self.get_logger().info(f'(gripper command {values[6]:.3f} ignored for now)')

        self.send_ros_goal(values[:6])

    def send_ros_goal(self, pose_delta):
        """ Publish the 6D pose delta to the Cartesian controller. """
        msg = Float64MultiArray()
        msg.data = pose_delta
        self.publisher_.publish(msg)
        self.get_logger().info(f'Relaying pose delta to ROS 2: {pose_delta}')

def main(args=None):
    rclpy.init(args=args)
    bridge_node = CartesianBridgeNode()
    try:
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        pass
    finally:
        bridge_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
