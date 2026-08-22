import sys
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class ModoClient(Node):
    def __init__(self):
        super().__init__("modo_client")
        self.cli = self.create_client(SetBool, "modo")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Aguardando modo_server...")


def main(args=None):
    rclpy.init(args=args)

    cli_args = [a for a in sys.argv[1:] if a != "--" and not a.startswith("--ros-args")]
    if not cli_args:
        print("Uso: ros2 run comunicacao_ros2 modo_client -- [true|false]")
        rclpy.shutdown()
        return  

    valor_desejado = cli_args[0].lower() in ("true", "1", "t", "yes")

    client = ModoClient()
    req = SetBool.Request()
    req.data = valor_desejado

    future = client.cli.call_async(req)
    rclpy.spin_until_future_complete(client, future)

    if future.result() is not None:
        print(future.result().message)
    else:
        client.get_logger().error(f"Falha na chamada de serviço: {future.exception()}")

    client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
