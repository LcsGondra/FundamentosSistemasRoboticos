import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Range
from std_msgs.msg import String
from std_srvs.srv import SetBool


class CoordenadorVigilancia(Node):
    def __init__(self):
        super().__init__("coordenador_vigilancia")
        self.lock = threading.Lock()
        self.alarme_ativo = False

        self.sub_dist = self.create_subscription(
            Range, "/distancia_frontal", self.distancia_callback, 10
        )
        self.sub_status = self.create_subscription(
            String, "/status_patrulha", self.status_callback, 10
        )
        self.srv_alarme = self.create_service(
            SetBool, "/ligar_alarme", self.ligar_alarme_callback
        )
        self.pub_alertas = self.create_publisher(String, "/alertas", 10)
        self.get_logger().info("No coordenador_vigilancia inicializado")

    def ligar_alarme_callback(self, request, response):
        with self.lock:
            self.alarme_ativo = request.data
            estado = "ativado" if self.alarme_ativo else "desativado"
        response.success = True
        response.message = f"Alarme {estado}"
        self.get_logger().info(response.message)
        return response

    def distancia_callback(self, msg):
        with self.lock:
            alarme_ligado = self.alarme_ativo

        if msg.range < 0.5 and alarme_ligado:
            alerta = String()
            alerta.data = "INTRUSO DETECTADO"
            self.pub_alertas.publish(alerta)
            self.get_logger().warn(
                f"INTRUSO DETECTADO! Distancia frontal: {msg.range:.2f} m"
            )

    def status_callback(self, msg):
        self.get_logger().info(f"Status da patrulha: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = CoordenadorVigilancia()
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
