import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Twist
from std_msgs.msg import Float32, String


class EstadoRobo(Node):
    def __init__(self):
        super().__init__("estado_robo")
        self.pub_pose = self.create_publisher(Pose2D, "/pose", 10)
        self.pub_vel = self.create_publisher(Twist, "/velocidade", 10)
        self.pub_bateria = self.create_publisher(Float32, "/bateria", 10)
        self.pub_status = self.create_publisher(String, "/status", 10)
        self.x = 0.0
        self.bateria = 100.0
        self.create_timer(1.0, self.tick)
        self.get_logger().info("No estado_robo inicializado")

    def tick(self):
        pose = Pose2D()
        self.x += 0.1
        pose.x = self.x
        self.pub_pose.publish(pose)

        vel = Twist()
        vel.linear.x = 0.3
        self.pub_vel.publish(vel)

        self.bateria = max(0.0, self.bateria - 2.0)
        bat_msg = Float32()
        bat_msg.data = self.bateria
        self.pub_bateria.publish(bat_msg)

        status = String()
        status.data = "bateria_critica" if self.bateria < 20.0 else "operando"
        self.pub_status.publish(status)

        self.get_logger().info(
            f"x: {self.x:.2f} | vel: {vel.linear.x} | bat: {self.bateria:.1f}% | status: {status.data}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = EstadoRobo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()