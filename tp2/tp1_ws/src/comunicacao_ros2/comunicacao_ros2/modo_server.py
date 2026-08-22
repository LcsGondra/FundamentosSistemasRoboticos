import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class ModoServer(Node):
    def __init__(self):
        super().__init__("modo_server")
        self.srv = self.create_service(SetBool, "modo", self.callback)
        self.get_logger().info(
            "Nó modo_server iniciado, aguardando chamadas no serviço /modo..."
        )

    def callback(self, request, response):
        if request.data:
            response.message = "Robô em modo autônomo ativado"
        else:
            response.message = "Robô em modo manual ativado"
        response.success = True
        self.get_logger().info(
            f"Requisição recebida ({request.data}) -> Resposta: {response.message}"
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ModoServer())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
