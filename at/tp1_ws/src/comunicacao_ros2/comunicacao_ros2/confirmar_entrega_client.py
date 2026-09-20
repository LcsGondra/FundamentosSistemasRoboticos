import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Trigger


class ConfirmarEntregaClient(Node):
    def __init__(self):
        super().__init__("confirmar_entrega_client")
        self.cli = self.create_client(Trigger, "/confirmar_entrega")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Aguardando servico /confirmar_entrega...")
        self.enviar_requisicao()

    def enviar_requisicao(self):
        req = Trigger.Request()
        self.future = self.cli.call_async(req)
        self.future.add_done_callback(self.resposta_callback)
        self.get_logger().info("Requisicao assincrona enviada via call_async.")

    def resposta_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(
                f"Resposta recebida de forma assincrona: Sucesso={response.success}, Mensagem='{response.message}'"
            )
        except Exception as e:
            self.get_logger().error(f"Falha na chamada de servico: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ConfirmarEntregaClient()
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
