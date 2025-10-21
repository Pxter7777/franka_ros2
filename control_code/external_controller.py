import socket
import time

# Some example joint configurations to send
M_PI_2 = 1.57079632679
M_PI_4 = 0.78539816339

SIGNALS = [
    [0.0, -M_PI_4, 0.0, -3.0 * M_PI_4, 0.0, M_PI_2, M_PI_4], # initial point
    [ -0.7462643384933472, 0.7409446835517883, 0.13806837797164917, -1.9485613107681274, 0.6339045763015747, 1.333835482597351, 1.1627459526062012],
    [ -0.7854958772659302, 0.2576063871383667, 0.29504284262657166, -2.3180482387542725, 0.8003225326538086, 1.2898585796356201, 1.2788596153259277],
    [ -0.6514967083930969, 0.5835818648338318, 0.3046169877052307, -1.8319802284240723, 0.885792076587677, 1.2863155603408813, 1.4982990026474],
    [0.0, -M_PI_4, 0.0, -3.0 * M_PI_4, 0.0, M_PI_2, M_PI_4] # initial point
]

class RobotCommander:
    """
    A class to manage connection and sending goals to the robot bridge.
    Connects automatically upon instantiation.
    """
    def __init__(self, host='localhost', port=9999):
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
            # Already connected or socket object exists, close it first to ensure a fresh connection
            self.disconnect()

        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.host, self.port))
            print(f"Connected to bridge at {self.host}:{self.port}")
            return True
        except ConnectionRefusedError:
            print(f"Connection failed. Is the bridge.py script running on {self.host}:{self.port}?")
            self.socket = None
            return False
        except Exception as e:
            print(f"An error occurred during connection: {e}")
            self.socket = None
            return False

    def disconnect(self):
        """
        Closes the connection to the robot bridge.
        """
        if self.socket:
            self.socket.close()
            self.socket = None
            print("Disconnected from bridge.")

    def send_goal(self, joint_positions):
        """
        Sends a single joint position goal to the robot bridge.
        Attempts to reconnect if the connection is lost.
        Returns True on successful send, False otherwise.
        """
        if not self.socket:
            print("Connection not established. Attempting to reconnect.")
            if not self._connect_on_init():  # Try to reconnect
                return False
        
        try:
            # Convert the list of numbers to a comma-separated string
            signal_str = ', '.join(map(str, joint_positions))
            
            print(f"Sending signal: {signal_str}")
            
            # Encode the string to bytes and send it
            self.socket.sendall(signal_str.encode('utf-8'))
            return True
        except BrokenPipeError:
            print("Connection lost while sending. Attempting to reconnect.")
            self.disconnect()
            if self._connect_on_init():  # Try to reconnect
                return self.send_goal(joint_positions)  # Retry sending
            return False
        except Exception as e:
            print(f"An error occurred while sending: {e}")
            return False

def main():
    commander = RobotCommander()  # Connection attempted here

    # Loop through the signals and send them
    for i, signal in enumerate(SIGNALS):
        print(f"Sending signal #{i+1}")
        if not commander.send_goal(signal):
            print(f"Failed to send signal #{i+1}. Aborting.")
            break
        time.sleep(1)  # Wait before sending the next one
    commander.disconnect()  # Disconnect when done
        
if __name__ == '__main__':
    main()