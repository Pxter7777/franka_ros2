import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import os
import socket
import sys
import threading
import time

class BridgeNode(Node):
    """
    This node acts as a bridge between a non-ROS application and the ROS 2 controller.
    It listens for TCP socket connections and translates received data into ROS 2 messages.
    """
    def __init__(self):
        super().__init__('pxter_bridge_node')
        
        # This publisher sends goals to the C++ controller
        self.publisher_ = self.create_publisher(Float64MultiArray, '/pxter_controller/goal', 10)
        self.get_logger().info('ROS 2 publisher on /pxter_controller/goal is ready.')

        # --- Socket Server Setup ---
        self.host = 'localhost'
        # Required env — forwarded from the superproject's
        # configs/network(.defaults).conf; deliberately no local fallback.
        if "FRANKA_JOINT_QUEUE_PORT" not in os.environ:
            sys.exit("FRANKA_JOINT_QUEUE_PORT is not set — pass docker exec -e FRANKA_JOINT_QUEUE_PORT=<port> (this bridge has no up-script)")
        self.port = int(os.environ["FRANKA_JOINT_QUEUE_PORT"])
        
        # The server must run in a separate thread to not block the ROS 2 node
        self.server_thread = threading.Thread(target=self._socket_server_loop)
        self.server_thread.daemon = True  # Allows the main program to exit even if this thread is running
        self.server_thread.start()

    def _socket_server_loop(self):
        self.get_logger().info(f'Socket server starting on {self.host}:{self.port}')
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((self.host, self.port))
            s.listen()
            
            # The server loop runs as long as the ROS 2 node is alive
            while rclpy.ok():
                try:
                    conn, addr = s.accept()
                    with conn:
                        self.get_logger().info(f'Socket connected by {addr}')
                        while True:
                            data = conn.recv(1024)
                            if not data:
                                break # Connection closed by client
                            try:
                                # Decode the received bytes into a string, and split by comma
                                joint_str = data.decode('utf-8').strip()
                                joint_positions = [float(p) for p in joint_str.split(',')]
                                
                                # Basic validation
                                if len(joint_positions) == 7:
                                    # If data is valid, publish it as a ROS 2 message
                                    self.send_ros_goal(joint_positions)
                                else:
                                    self.get_logger().warn(f'Received invalid data: needs 7 joints, got {len(joint_positions)}')
                            except Exception as e:
                                self.get_logger().error(f'Failed to process received data: {e}')
                except Exception as e:
                    self.get_logger().error(f'Socket server error: {e}')
                    time.sleep(1) # Avoid busy-looping on error

    def send_ros_goal(self, joint_positions):
        """ Publishes the goal to the ROS 2 topic. """
        msg = Float64MultiArray()
        msg.data = joint_positions
        self.publisher_.publish(msg)
        self.get_logger().info(f'Relaying goal to ROS 2: {joint_positions}')

def main(args=None):
    rclpy.init(args=args)
    bridge_node = BridgeNode()
    
    # rclpy.spin() keeps the node alive and processing ROS events.
    # The socket server is running in a background thread.
    try:
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        bridge_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()