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
        if i >= 0:
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
            if i >= 0:
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
            
            try:
                data = ser.readline().decode('utf-8')
                data = [float(x) for x in data.split(',')[:7]]
            
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.accelerometer_x = data[0]
                msg.accelerometer_y = data[1]
                msg.accelerometer_z = data[2]
                msg.front_left_wheel_velocity = data[4]
                msg.front_right_wheel_velocity = data[6]
                msg.rear_left_wheel_velocity = data[5]
                msg.rear_right_wheel_velocity = data[3]
            
                publisher.publish(msg)
                
            except Exception as e:
                pass

    except KeyboardInterrupt:
        print("Exiting program")
        ser.close()

