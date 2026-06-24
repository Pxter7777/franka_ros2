import socket
import time

# Some example end-effector pose DELTAS to send (base frame).
# [dx, dy, dz, droll, dpitch, dyaw] -- translation in metres, rotation in radians.
# Each one is applied relative to the current target, so they accumulate.
SIGNALS = [
    [0.05, 0.0, 0.0, 0.0, 0.0, 0.0],   # +5 cm in x
    [-0.05, 0.0, 0.0, 0.0, 0.0, 0.0],  # back -5 cm in x
    [0.0, 0.05, 0.0, 0.0, 0.0, 0.0],   # +5 cm in y
    [0.0, -0.05, 0.0, 0.0, 0.0, 0.0],  # back -5 cm in y
    [0.0, 0.0, 0.05, 0.0, 0.0, 0.0],   # +5 cm in z
    [0.0, 0.0, -0.05, 0.0, 0.0, 0.0],  # back -5 cm in z
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.2],    # +0.2 rad yaw
    [0.0, 0.0, 0.0, 0.0, 0.0, -0.2],   # back -0.2 rad yaw
]

class CartesianCommander:
    """
    A class to manage connection and sending pose deltas to the cartesian bridge.
    Connects automatically upon instantiation.
    """
    def __init__(self, host='localhost', port=9998):
        self.host = host
        self.port = port
        self.socket = None
        self._connect_on_init()  # Attempt connection during initialization

    def _connect_on_init(self):
        """
        Internal method to establish connection. Used during init and for re-connection.
        Returns True on success, False otherwise.
        """
        if self.socket:
            self.disconnect()

        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.host, self.port))
            print(f"Connected to cartesian bridge at {self.host}:{self.port}")
            return True
        except ConnectionRefusedError:
            print(f"Connection failed. Is cartesian_bridge.py running on {self.host}:{self.port}?")
            self.socket = None
            return False
        except Exception as e:
            print(f"An error occurred during connection: {e}")
            self.socket = None
            return False

    def disconnect(self):
        """
        Closes the connection to the cartesian bridge.
        """
        if self.socket:
            self.socket.close()
            self.socket = None
            print("Disconnected from cartesian bridge.")

    def send_goal(self, pose_delta):
        """
        Sends a single 6D pose delta to the cartesian bridge.
        Attempts to reconnect if the connection is lost.
        Returns True on successful send, False otherwise.
        """
        if not self.socket:
            print("Connection not established. Attempting to reconnect.")
            if not self._connect_on_init():
                return False

        try:
            # Comma-separated, newline-terminated (the bridge frames on '\n').
            signal_str = ', '.join(map(str, pose_delta)) + '\n'

            print(f"Sending pose delta: {signal_str.strip()}")

            self.socket.sendall(signal_str.encode('utf-8'))
            return True
        except BrokenPipeError:
            print("Connection lost while sending. Attempting to reconnect.")
            self.disconnect()
            if self._connect_on_init():
                return self.send_goal(pose_delta)  # Retry sending
            return False
        except Exception as e:
            print(f"An error occurred while sending: {e}")
            return False

def main():
    commander = CartesianCommander()  # Connection attempted here

    # Loop through the deltas and send them. Wait between sends so each move has
    # time to play out (the controller ramps slowly under its velocity limits).
    for i, signal in enumerate(SIGNALS):
        print(f"Sending delta #{i+1}")
        if not commander.send_goal(signal):
            print(f"Failed to send delta #{i+1}. Aborting.")
            break
        time.sleep(3)  # Wait before sending the next one
    commander.disconnect()

if __name__ == '__main__':
    main()
