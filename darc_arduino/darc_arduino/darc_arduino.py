import rclpy
from darc_interfaces.msg import DarcArduino
import serial
import struct

# Faster pyserial readline
# https://github.com/pyserial/pyserial/issues/216#issuecomment-369414522
class ReadLine:
    def __init__(self, s):
        self.buf = bytearray()
        self.s = s
    
    def readline(self):
        i = self.buf.find(b"\n")
        if i >= 1 and self.buf[i-1] == b"\r":
            r = self.buf[:i+1]
            self.buf = self.buf[i+1:]
            return r
        while True:
            i = max(1, min(2048, self.s.in_waiting))
            data = self.s.read(i)

            # Check if timed out
            if data == "":
                return bytearray()
            
            i = data.find(b"\n")
            if i >= 1 and self.buf[i-1] == b"\r":
                r = self.buf + data[:i+1]
                self.buf[0:] = data[i+1:]
                return r
            else:
                self.buf.extend(data)
                
def main(args=None):
    rclpy.init(args=args)
    
    node = rclpy.create_node('darc_arduino')
    publisher = node.create_publisher(DarcArduino, 'arduino', 10)
    
    node.declare_parameter('com_port', '/dev/ttyACM1')
    port = node.get_parameter('com_port').value
    
    msg = DarcArduino()
    
    # Configure the serial port
    ser = serial.Serial(port, 9600)  # Check the correct port
    reader = ReadLine(ser)
    
    print('starting to read port on', port)
    
    try:
        while rclpy.ok():
            data = ser.readline()
            #print("Raw data:", data)  # See exactly what is being received        
            if data[0:1] != b'\xAA':         
                #   print("START BIT NOT FOUND", data[0:1])
                continue
            if data[-2:] != b'\r\n':
                #  print("END BIT NOT FOUND", data[-2:])
                continue
            if len(data) != 31:
                # print("DATA LENGTH WRONG", len(data))
                continue
            
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.accelerometer_x = struct.unpack('<f', data[1:5])[0]
            msg.accelerometer_y = struct.unpack('<f', data[5:9])[0]
            msg.accelerometer_z = struct.unpack('<f', data[9:13])[0]
            msg.front_left_wheel_velocity = struct.unpack('<f', data[17:21])[0]
            msg.front_right_wheel_velocity = struct.unpack('<f', data[25:29])[0]
            msg.rear_left_wheel_velocity = struct.unpack('<f', data[21:25])[0]
            msg.rear_right_wheel_velocity = struct.unpack('<f', data[13:17])[0] 
            
            publisher.publish(msg)


    except KeyboardInterrupt:
        print("Exiting program")
        ser.close()

