import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from example_interfaces.action import Fibonacci


class TransporteActionClient(Node):
    def __init__(self):
        super().__init__("transporte_action_client")
        self._action_client = ActionClient(self, Fibonacci, "transporte_leito")
        self._goal_handle = None

    def enviar_meta(self, order):
        self.get_logger().info("Aguardando Action Server...")
        self._action_client.wait_for_server()

        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self.get_logger().info(f"Enviando meta com {order} etapas...")
        send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.meta_resposta_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Feedback intermediario recebido: {feedback.sequence}")

    def meta_resposta_callback(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().warn("Meta rejeitada pelo servidor.")
            return

        self.get_logger().info("Meta aceita pelo servidor. Aguardando resultado...")
        get_result_future = self._goal_handle.get_result_async()
        get_result_future.add_done_callback(self.resultado_callback)

    def cancelar_meta(self):
        if self._goal_handle:
            self.get_logger().info("Solicitando cancelamento da meta...")
            self._goal_handle.cancel_goal_async()

    def resultado_callback(self, future):
        result = future.result().result
        status = future.result().status
        self.get_logger().info(f"Resultado final recebido (Status {status}): {result.sequence}")


def main(args=None):
    rclpy.init(args=args)
    node = TransporteActionClient()
    node.enviar_meta(5)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
