import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist

class MonitorVelocidade(Node):
    def __init__(self):
        super().__init__('monitor_velocidade')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Twist, '/cmd_vel', self.callback, qos)
    
    def callback(self, msg):
        v = math.sqrt(msg.linear.x**2 + msg.linear.y**2 + msg.linear.z**2)
        sentido = 'horário' if msg.angular.z < 0 else 'anti-horário'
        self.get_logger().info(f'v={v:.2f} m/s, sentido={sentido}')

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MonitorVelocidade())
    rclpy.shutdown()