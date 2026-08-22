import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class VelocidadePub(Node):
    def __init__(self):
        super().__init__("velocidade_pub")
        self.pub = self.create_publisher(Twist, "/robo1/cmd_vel", 10)
        self.create_timer(0.5, self.tick)

    def tick(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.5
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(VelocidadePub())
    rclpy.shutdown()
