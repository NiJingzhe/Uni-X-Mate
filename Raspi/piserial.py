import serial

class piSerial:

    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.ser = serial.Serial(port, baudrate, timeout=500)

    def read(self, size):
        return self.ser.read(size)
    
    def readline(self):
        return self.ser.readline()

    def write(self, data):
        self.ser.write(data)

    def close(self):
        self.ser.close()    
        
    def reset_input_buffer(self):
        self.ser.reset_input_buffer()