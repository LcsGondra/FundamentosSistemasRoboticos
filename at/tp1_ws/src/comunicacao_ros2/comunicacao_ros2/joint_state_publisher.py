import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStatePublisher(Node):

    def __init__(self):
        super().__init__("joint_state_publisher")
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["joint_roda_esq", "joint_roda_dir"]
        msg.position = [0.0, 0.0]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
