import random
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Temperature
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class MonitorTransporte(Node):
    def __init__(self):
        super().__init__("monitor_transporte")
        self.declare_parameter("temperatura_limite", 28.0)
        self.add_on_set_parameters_callback(self.param_callback)

        self.cb_group = ReentrantCallbackGroup()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        critical_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.pub_temp = self.create_publisher(
            Temperature, "/temperatura_ambiente", sensor_qos
        )
        self.pub_ocupado = self.create_publisher(
            Bool, "/robo_ocupado", critical_qos
        )
        self.pub_alertas = self.create_publisher(
            String, "/alertas", critical_qos
        )

        self.srv_entrega = self.create_service(
            Trigger,
            "/confirmar_entrega",
            self.confirmar_entrega_callback,
            callback_group=self.cb_group,
        )

        self.ocupado = True
        self.timer = self.create_timer(1.0, self.timer_callback, callback_group=self.cb_group)
        self.get_logger().info("No monitor_transporte inicializado com QoS configurado.")

    def param_callback(self, params):
        for param in params:
            if param.name == "temperatura_limite":
                self.get_logger().info(f"Parametro temperatura_limite alterado para: {param.value}")
        return SetParametersResult(successful=True)

    def timer_callback(self):
        temp_val = round(random.uniform(18.0, 30.0), 2)
        temp_msg = Temperature()
        temp_msg.temperature = float(temp_val)
        self.pub_temp.publish(temp_msg)

        ocupado_msg = Bool()
        ocupado_msg.data = self.ocupado
        self.pub_ocupado.publish(ocupado_msg)

        limite = self.get_parameter("temperatura_limite").get_parameter_value().double_value

        self.get_logger().info(
            f"Temperatura: {temp_val:.2f} C (Limite: {limite:.2f} C) | Ocupado: {self.ocupado}"
        )

        if temp_val > limite:
            alerta_msg = String()
            alerta_msg.data = "TEMPERATURA FORA DO PADRAO"
            self.pub_alertas.publish(alerta_msg)
            self.get_logger().warn(f"Alerta publicado: {alerta_msg.data}")

    def confirmar_entrega_callback(self, request, response):
        self.ocupado = False

        alerta_msg = String()
        alerta_msg.data = "Entrega confirmada"
        self.pub_alertas.publish(alerta_msg)

        ocupado_msg = Bool()
        ocupado_msg.data = self.ocupado
        self.pub_ocupado.publish(ocupado_msg)

        response.success = True
        response.message = "Entrega confirmada"
        self.get_logger().info("Servico /confirmar_entrega executado: entrega confirmada.")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MonitorTransporte()
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
