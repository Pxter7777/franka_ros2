import socket
import time

# This is a simple, non-ROS Python script that acts as the external controller.

# The host and port must match the server in bridge.py
HOST = 'localhost'
PORT = 9999

# Some example joint configurations to send
M_PI_2 = 1.57079632679
M_PI_4 = 0.78539816339

SIGNALS = [
    [0.0, -M_PI_4, 0.0, -3.0 * M_PI_4, 0.0, M_PI_2, M_PI_4],
    [0.0, 0.0, 0.0, -3.0 * M_PI_4, 0.0, M_PI_2, M_PI_4],
    [0.0, -M_PI_4, 0.0, 0.0, 0.0, M_PI_2, M_PI_4],
    [0.0, -M_PI_4, 0.0, -3.0 * M_PI_4, 0.0, M_PI_2, 0.0]
]

def main():
    print("--- External Controller ---")
    print(f"Attempting to connect to bridge at {HOST}:{PORT}")

    try:
        # Create a standard TCP socket client
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((HOST, PORT))
            print("Connected to bridge. Sending goals...")

            # Loop through the list of signals
            for i, signal in enumerate(SIGNALS):
                # Convert the list of numbers to a comma-separated string
                signal_str = ', '.join(map(str, signal))    
                
                print(f"Sending signal #{i+1}: {signal_str}")
                
                # Encode the string to bytes and send it over the socket
                s.sendall(signal_str.encode('utf-8'))
                
                # Wait before sending the next one
                time.sleep(5)
            
            print("All signals sent. Closing connection.")

    except ConnectionRefusedError:
        print(f"Connection failed. Is the bridge.py script running?")
    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == '__main__':
    main()
