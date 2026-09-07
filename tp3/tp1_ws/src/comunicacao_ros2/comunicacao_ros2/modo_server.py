import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class ModoServer(Node):
    def __init__(self):
        super().__init__('modo_server')
        self.create_service(SetBool, 'modo', self.callback)
        self.get_logger().info('nó iniciado, aguardando chamadas em /modo')

    def callback(self, request, response):
        if request.data:
            response.message = 'Robô em modo autônomo ativado'
        else:
            response.message = 'Robô em modo manual ativado'
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ModoServer())
    rclpy.shutdown()