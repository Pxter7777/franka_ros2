import os
import socket
import sys
import threading
import time

HOST = 'localhost'
# Required env — forwarded from the superproject's
# configs/network(.defaults).conf; deliberately no local fallback.
if "FRANKA_JOINT_QUEUE_PORT" not in os.environ:
    sys.exit("FRANKA_JOINT_QUEUE_PORT is not set — set FRANKA_JOINT_QUEUE_PORT in the environment (dummy bridge, host-run)")
PORT = int(os.environ["FRANKA_JOINT_QUEUE_PORT"])

def handle_client(conn, addr):
    print(f"Dummy Bridge: Connected by {addr}")
    while True:
        data = conn.recv(1024)
        if not data:
            break
        try:
            joint_str = data.decode('utf-8').strip()
            joint_positions = [float(p) for p in joint_str.split(',')]

            if len(joint_positions) == 7:
                print(f"Dummy Bridge: Received valid goal: {joint_positions}")
            else:
                print(f"Dummy Bridge: Received invalid data format: needs 7 joints, got {len(joint_positions)} - Data: '{joint_str}'")
        except ValueError:
            print(f"Dummy Bridge: Received non-numeric data: '{joint_str}'")
        except Exception as e:
            print(f"Dummy Bridge: Error processing data: {e}")
    print(f"Dummy Bridge: Client {addr} disconnected.")

def main():
    print(f"Dummy Bridge: Starting server on {HOST}:{PORT}")
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen()
        print("Dummy Bridge: Waiting for connections...")
        while True:
            conn, addr = s.accept()
            client_thread = threading.Thread(target=handle_client, args=(conn, addr))
            client_thread.daemon = True
            client_thread.start()

if __name__ == '__main__':
    main()