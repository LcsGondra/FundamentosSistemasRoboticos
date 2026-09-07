import sys
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci


class PatrulhaClient(Node):
    def __init__(self):
        super().__init__("patrulha_client")
        self._client = ActionClient(self, Fibonacci, "patrulha")
        self.n = 3

    def send_goal(self, n):
        self.n = n
        goal_msg = Fibonacci.Goal()
        goal_msg.order = n

        self.get_logger().info("Aguardando Action Server patrulha...")
        self._client.wait_for_server()

        future = self._client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejeitado pelo servidor")
            rclpy.shutdown()
            return

        self.get_logger().info("Goal aceito pelo servidor")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        n_visitados = len(feedback_msg.feedback.sequence)
        self.get_logger().info(f"Visitando ponto {n_visitados} de {self.n}")

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Resultado final recebido: {result.sequence}")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    client = PatrulhaClient()

    target_points = 3
    if len(sys.argv) > 1 and sys.argv[1].isdigit():
        target_points = int(sys.argv[1])

    client.send_goal(target_points)

    try:
        rclpy.spin(client)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    from rclpy.executors import ExternalShutdownException

    main()