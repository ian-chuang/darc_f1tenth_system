import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped

class VelocityCalculator(Node):
    def __init__(self):
        super().__init__('velocity_calculator')
        
        # Subscribing to LIDAR odometry values
        self.subscription = self.create_subscription(
            Odometry, '/pf/pose/odom', self.listener_callback, 10)
            
        print('Subscribing to /pf/pose/odom...')
        
        # Publisher for LIDAR based velocity
        self.publisher = self.create_publisher(TwistStamped, '/LIDAR_velocity', 10)
        
        # Initialize last position, last time, and velocity timestamp
        self.last_pose = None
        self.last_time = None
        self.velocities = []
        
        # Averaging 5 velocities at a time
        self.window_size = 5
        
    def listener_callback(self, msg):
        # Creating current position and current time variables from the LIDAR odometry
        current_pose = msg.pose.pose
        current_time = msg.header.stamp
        
        # Calculating change in position and change in time
        if self.last_pose is not None and self.last_time is not None:
            dx = current_pose.position.x - self.last_pose.position.x
            dy = current_pose.position.y - self.last_pose.position.y
            dt = (current_time.sec - self.last_time.sec) + (current_time.nanosec - self.last_time.nanosec) * 1e-9
            
            if dt > 0:
                velocity = ((dx**2 + dy**2)**0.5) / dt
                self.velocities.append(velocity)
                
                if len(self.velocities) > self.window_size:
                    self.velocities.pop(0)
                    
                avg_velocity = sum(self.velocities) / len(self.velocities)
                
                vel_msg = TwistStamped()
                vel_msg.header.stamp = self.get_clock().now().to_msg()
                vel_msg.twist.linear.x = avg_velocity
                self.publisher.publish(vel_msg)
                
        self.last_pose = current_pose
        self.last_time = current_time
        
def main(args=None):
    rclpy.init(args=args)
    velocity_calculator = VelocityCalculator()
    rclpy.spin(velocity_calculator)
    
    velocity_calculator.destroy_node()
    rclpy.shutdown()
    
if __name__ == 'main':
    main()   
