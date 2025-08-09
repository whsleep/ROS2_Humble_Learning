import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TrajNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        self.radius = 2.0 
        self.speed = 0.5 
        
        self.angular_velocity = self.speed / self.radius
        
        # Create a timer to publish velocity commands at 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info(f"Starting circular motion with radius {self.radius} m and speed {self.speed} m/s")

    def timer_callback(self):
        twist_msg = Twist()
        
        twist_msg.linear.x = self.speed
        
        twist_msg.angular.z = self.angular_velocity
        
        self.publisher.publish(twist_msg)


def main():
    rclpy.init()
    node = TrajNode("circular_trajectory_node")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()