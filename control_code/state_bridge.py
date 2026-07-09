import rclpy
from rclpy.node import Node
from franka_msgs.msg import FrankaRobotState
import socket
import threading
import time

class StateBridgeNode(Node):
    """
    Serves the latest Franka robot state to a non-ROS application over TCP.

    Counterpart of joint_stream_bridge.py, but in the opposite direction: instead
    of relaying goals in, it polls state out (needed for closed-loop GR00T, whose
    observation includes the current joints and EE pose).

    Protocol: request/response. The client sends any newline-terminated line
    (e.g. "get\\n"); the server replies with one line of 14 comma-separated floats:
        q1..q7, px, py, pz, qx, qy, qz, qw
    (measured joint positions in radians; O_T_EE position in meters and
    orientation quaternion in x,y,z,w order). Before the first state message
    arrives the reply is "nodata".

    The broadcaster publishes ~/robot_state inside the robot's namespace from
    franka_bringup/config/franka.config.yaml (default "NS_1"). Override with:
        python3 src/control_code/state_bridge.py --ros-args -p topic:=/...
    """
    def __init__(self):
        super().__init__('pxter_state_bridge_node')

        self.state_lock = threading.Lock()
        self.latest_line = None

        self.topic = self.declare_parameter(
            'topic', '/NS_1/franka_robot_state_broadcaster/robot_state').value
        self.subscription = self.create_subscription(
            FrankaRobotState,
            self.topic,
            self._state_callback,
            10)
        self.get_logger().info(f'Subscribed to {self.topic}.')

        self.host = 'localhost'
        self.port = 9996  # 9999/9998/9997 = command bridges; 9996 = state out.

        self.server_thread = threading.Thread(target=self._socket_server_loop)
        self.server_thread.daemon = True
        self.server_thread.start()

        # Catch "broadcaster not launched" / namespace mismatches early.
        self.startup_timer = self.create_timer(5.0, self._warn_if_no_state)

    def _warn_if_no_state(self):
        self.startup_timer.cancel()
        with self.state_lock:
            have_state = self.latest_line is not None
        if not have_state:
            self.get_logger().warn(
                f'No FrankaRobotState received after 5 s on {self.topic}. Is the '
                'controller stack up? Compare against `ros2 topic list | grep '
                'robot_state` — if the namespace differs, rerun with '
                '--ros-args -p topic:=/<ns>/franka_robot_state_broadcaster/robot_state')

    def _state_callback(self, msg):
        q = list(msg.measured_joint_state.position)[:7]
        if len(q) < 7:
            self.get_logger().warn(
                f'measured_joint_state has only {len(q)} positions; skipping.')
            return
        pos = msg.o_t_ee.pose.position
        ori = msg.o_t_ee.pose.orientation
        values = q + [pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w]
        line = ','.join(f'{v:.6f}' for v in values) + '\n'
        with self.state_lock:
            self.latest_line = line

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
                                _, buffer = buffer.split('\n', 1)
                                with self.state_lock:
                                    reply = self.latest_line
                                if reply is None:
                                    reply = 'nodata\n'
                                conn.sendall(reply.encode('utf-8'))
                except Exception as e:
                    self.get_logger().error(f'Socket server error: {e}')
                    time.sleep(1)  # Avoid busy-looping on error

def main(args=None):
    rclpy.init(args=args)
    bridge_node = StateBridgeNode()
    try:
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        pass
    finally:
        bridge_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
