import time
import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from example_interfaces.action import Fibonacci


class PatrulhaServer(Node):
    def __init__(self):
        super().__init__("patrulha_server")
        self._action_server = ActionServer(
            self, Fibonacci, "patrulha", self.execute_callback
        )
        self.get_logger().info("Action Server patrulha_server inicializado")

    def execute_callback(self, goal_handle):
        n = goal_handle.request.order
        self.get_logger().info(f"Executando rota com {n} pontos")
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = []

        for i in range(1, n + 1):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().warn("Patrulha cancelada")
                result = Fibonacci.Result()
                result.sequence = feedback_msg.sequence
                return result

            time.sleep(1.0)
            feedback_msg.sequence.append(i)
            self.get_logger().info(f"Visitando ponto {i} de {n}")
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        self.get_logger().info("Patrulha concluida com sucesso")
        return result


def main(args=None):
    rclpy.init(args=args)
    node = PatrulhaServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()