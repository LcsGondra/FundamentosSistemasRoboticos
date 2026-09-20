import time
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.node import Node
from example_interfaces.action import Fibonacci


class TransporteActionServer(Node):
    def __init__(self):
        super().__init__("transporte_action_server")
        self._action_server = ActionServer(
            self,
            Fibonacci,
            "transporte_leito",
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
        )
        self.get_logger().info("Action Server transporte_leito inicializado.")

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Recebido pedido de cancelamento da meta de transporte.")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        n = goal_handle.request.order
        self.get_logger().info(f"Iniciando rota de transporte hospitalar com {n} etapas.")
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = []

        for i in range(1, n + 1):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().warn(f"Transporte cancelado preventivamente no leito {i}.")
                result = Fibonacci.Result()
                result.sequence = feedback_msg.sequence
                return result

            time.sleep(1.0)
            feedback_msg.sequence.append(i)
            self.get_logger().info(f"Progresso: atendendo leito {i} de {n}.")
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        self.get_logger().info("Rota de transporte concluida com sucesso.")
        return result


def main(args=None):
    rclpy.init(args=args)
    node = TransporteActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
