import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class VelocityCalculator(Node):
    def __init__(self):
        super().__init__('velocity_calculator')
        self.subscription = self.create_subscription(
            Odometry, '/pf/pose/odom', self.listener_callback, 10)
        self.last_pose = None
        self.last_time = None
        self.velocities = []
        #averaging 10 velocities at a time
        self.window_size = 10
        
        print("subscribing to /odom")
        
    def listener_callback(self, msg):
    
        print("received msg")
        current_pose = msg.pose.pose
        current_time = msg.header.stamp
        
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
                print(avg_velocity)
                #self.get_logger().info(f'Timestamp: {current.time.sec}.{current_time.nanosec}, Average Velocity: {avg_velocity:.2f} m/s')
                
        self.last_pose = current_pose
        self.last_time = current_time
            
def main(args=None):
    print("lsdjflsjfdlk")
    print("lsdjflsjfdlk")
    rclpy.init(args=args)
    print("lsdjflsjfdlk")
    velocity_calculator = VelocityCalculator()
    rclpy.spin(velocity_calculator)
    
    velocity_calculator.destroy_node()
    rclpy.shutdown()
    
if __name__ == 'main':
    main()        
