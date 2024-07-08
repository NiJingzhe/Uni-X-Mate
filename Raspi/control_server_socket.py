import socket
import threading
import struct
from enum import Enum
from piserial import piSerial
import serial

# Constants
HOST = '0.0.0.0'  # Listen on all interfaces
PORT = 5001       # Port for socket server

SERIAL_PORT = '/dev/ttyACM0'
SERIAL_BAUDRATE = 9600

# Enum for commands
class COMMANDS(Enum):
    MOVE = 0
    IMM_STOP = 1

# Initialize serial
try:
    pi_serial = piSerial(SERIAL_PORT, SERIAL_BAUDRATE)
    print("串口通讯成功打开")
    SERIAL_ENABLE = True
except Exception as e:
    print(f"Error initializing serial port: {e}")
    SERIAL_ENABLE = False

def int_to_signed_byte(value):
    """Convert an integer to a signed byte."""
    return struct.pack('b', value)

def handle_client_connection(client_socket):
    """Handle incoming socket connection."""
    try:
        while True:
            message = client_socket.recv(1024).decode('utf-8')
            if not message:
                break
            
            # Parse the message
            forward_back, left_right = map(int, message.strip().split(','))

            # Build command byte stream
            command_byte = COMMANDS.MOVE.value.to_bytes(1, 'big')
            forward_back_byte = struct.pack('b', forward_back)
            left_right_byte = struct.pack('b', left_right)
            command_stream_bytes = command_byte + forward_back_byte + left_right_byte

            try:
                if SERIAL_ENABLE:
                    pi_serial.write(command_stream_bytes)
                    feed_back = pi_serial.readline().decode().strip()
                    client_socket.sendall(feed_back.encode('utf-8'))
                else:
                    client_socket.sendall(b"Serial not enabled")
            except serial.SerialTimeoutException:
                client_socket.sendall(b"Serial timeout occurred")
            except serial.SerialException as e:
                client_socket.sendall(f"Serial communication error: {e}".encode('utf-8'))
            except Exception as e:
                client_socket.sendall(f"Unexpected error: {e}".encode('utf-8'))

    finally:
        client_socket.close()

def start_socket_server():
    """Start the socket server."""
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((HOST, PORT))
    server_socket.listen(5)
    print(f"Socket server listening on {HOST}:{PORT}")
    
    while True:
        client_socket, addr = server_socket.accept()
        print(f"Accepted connection from {addr}")
        client_handler = threading.Thread(target=handle_client_connection, args=(client_socket,))
        client_handler.start()
