import threading
# pyrefly: ignore [missing-import]
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import SetBool
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


class Controlador(Node):
    def __init__(self):
        super().__init__("controlador")
        self.lock = threading.Lock()
        self.estado = "parado"
        self.srv = self.create_service(
            SetBool, "/mudar_estado", self.mudar_estado_callback
        )
        self.sub_bateria = self.create_subscription(
            Float32, "/bateria", self.bateria_callback, 10
        )
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_timer(0.5, self.publicar_cmd_vel)
        self.get_logger().info("No controlador inicializado")

    def mudar_estado_callback(self, request, response):
        with self.lock:
            if self.estado == "emergencia":
                response.success = False
                response.message = "Robo em emergencia, bateria critica. Comando ignorado."
            else:
                self.estado = "movendo" if request.data else "parado"
                response.success = True
                response.message = f"Estado alterado para {self.estado}"
        self.get_logger().info(response.message)
        return response

    def bateria_callback(self, msg):
        with self.lock:
            if msg.data < 20.0 and self.estado != "emergencia":
                self.estado = "emergencia"
                self.get_logger().warn(
                    f"BATERIA CRITICA {msg.data:.1f}%: Transicao para emergencia"
                )

    def publicar_cmd_vel(self):
        with self.lock:
            estado_atual = self.estado
        msg = Twist()
        msg.linear.x = 0.5 if estado_atual == "movendo" else 0.0
        self.pub_cmd.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Controlador()
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