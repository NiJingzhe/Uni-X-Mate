import socket
import threading
import struct
from enum import Enum
from piserial import piSerial
import serial
import json

# Constants
HOST = '0.0.0.0'  # Listen on all interfaces
PORT = 5001       # Port for socket server

SERIAL_PORT = 'com6'
SERIAL_BAUDRATE = 115200

# Enum for commands
class COMMANDS(Enum):
    MOVE = 0
    IMM_STOP = 1

# Initialize serial
try:
    pi_serial = piSerial(SERIAL_PORT, SERIAL_BAUDRATE)
    print("====="*5)
    print("串口通讯成功打开")
    print("====="*5)
    SERIAL_ENABLE = True
except Exception as e:
    print(f"Error initializing serial port: {e}")
    SERIAL_ENABLE = False
    
def handle_client_connection(client_socket):
    """Handle incoming socket connection."""
    try:
        while True:
            message = client_socket.recv(1024)
            if not message:
                break
            
            print("遥控服务端收到: ", message)

            command_stream_bytes = message
            
            feed_back = {"self_check_result" : 1}
            client_socket.sendall(json.dumps(feed_back).encode('utf-8'))

    finally:
        client_socket.close()


def control_socket_start():
    """Start the socket server."""
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((HOST, PORT))
    server_socket.listen(5)
    print(f"Socket server listening on {HOST}:{PORT}")

    while True:
        try:
            client_socket, addr = server_socket.accept()
            print(f"Accepted connection from {addr}")
            client_handler = threading.Thread(target=handle_client_connection, args=(client_socket,))
            client_handler.start()
        except Exception as e:
            print(f"Error accepting connection: {e}")