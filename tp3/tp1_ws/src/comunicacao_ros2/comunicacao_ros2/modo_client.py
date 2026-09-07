import sys
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class ModoClient(Node):
    def __init__(self):
        super().__init__('modo_client')
        self.cli = self.create_client(SetBool, 'modo')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('aguardando modo_server...')


def main(args=None):
    rclpy.init(args=args)
    client = ModoClient()
    req = SetBool.Request()
    req.data = sys.argv[1].lower() == 'true'
    future = client.cli.call_async(req)
    rclpy.spin_until_future_complete(client, future)
    print(future.result().message)
    rclpy.shutdown()