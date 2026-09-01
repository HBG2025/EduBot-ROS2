import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import serial
import time

class SerialBridge(Node):

    def __init__(self):
        super().__init__('serial_bridge')

        # ==========================
        # PARÁMETROS SERIAL
        # ==========================
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)

        puerto = self.get_parameter('port').value
        baud = self.get_parameter('baud').value

        # ==========================
        # ABRIR PUERTO SERIAL
        # ==========================
        try:
            self.serial_port = serial.Serial(
                port=puerto,
                baudrate=baud,
                timeout=0.05,
                dsrdtr=False,
                rtscts=False
            )

            self.serial_port.dtr = False
            self.serial_port.rts = False
            
            time.sleep(2.0)

            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()

            self.get_logger().info(
                f'Conectado al ESP32 en {puerto} a {baud} baudios'
            )

        except serial.SerialException as e:

            self.get_logger().error(
                f'No se pudo abrir el puerto serial: {e}'
            )

            self.serial_port = None

        # ==========================
        # SUSCRIPTOR ROS 2
        # ==========================
        self.suscriptor_comando = self.create_subscription(
            String,
            '/robot/comando',
            self.recibir_comando,
            10
        )

        # ==========================
        # TIMER PARA LEER ESP32
        # ==========================
        self.timer = self.create_timer(
            0.05,
            self.leer_serial
        )

        self.get_logger().info(
            'Esperando comandos en /robot/comando'
        )

    # ==========================
    # ROS 2 -> ESP32
    # ==========================
    def recibir_comando(self, msg):

        if self.serial_port is None:
            self.get_logger().error(
                'Puerto serial no disponible'
            )
            return

        comando = msg.data.strip()

        if not comando:
            return

        try:

            mensaje = comando + '\n'

            self.serial_port.write(
                mensaje.encode('utf-8')
            )

            self.get_logger().info(
                f'ROS2 -> ESP32: {comando}'
            )

        except serial.SerialException as e:

            self.get_logger().error(
                f'Error enviando comando: {e}'
            )

    # ==========================
    # ESP32 -> ROS 2
    # ==========================
    def leer_serial(self):

        if self.serial_port is None:
            return

        try:

            while self.serial_port.in_waiting > 0:

                linea = self.serial_port.readline()

                try:
                    texto = linea.decode('utf-8').strip()

                except UnicodeDecodeError:
                    self.get_logger().warning(
                    'Trama serial corrupta descartada'
                    )
                    return
                
                if texto:

                    self.get_logger().info(
                        f'ESP32 -> {texto}'
                    )

        except serial.SerialException as e:

            self.get_logger().error(
                f'Error leyendo puerto serial: {e}'
            )

    # ==========================
    # CIERRE
    # ==========================
    def destroy_node(self):

        if self.serial_port is not None:
            self.serial_port.close()

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    nodo = SerialBridge()

    try:

        rclpy.spin(nodo)

    except KeyboardInterrupt:

        pass

    nodo.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
