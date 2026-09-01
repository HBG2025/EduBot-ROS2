import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32

import ydlidar
import math


class LidarNode(Node):

    def __init__(self):

        super().__init__('lidar_node')

        # =====================================================
        # PUBLICADOR ROS 2
        # =====================================================

        self.publicador_distancia = self.create_publisher(
            Float32,
            '/lidar/distancia',
            10
        )

        # =====================================================
        # PUERTO FÍSICO DEL YDLIDAR X4
        # =====================================================
        #
        # Se usa by-path para evitar confundirlo con el ESP32.

        self.puerto = (
            '/dev/serial/by-path/'
            'platform-xhci-hcd.0-usb-0:1:1.0-port0'
        )

        self.get_logger().info(
            f'Puerto YDLIDAR: {self.puerto}'
        )

        # =====================================================
        # INICIALIZACIÓN DEL SDK
        # =====================================================

        ydlidar.os_init()

        self.laser = ydlidar.CYdLidar()

        # =====================================================
        # CONFIGURACIÓN VALIDADA DEL X4
        # =====================================================

        self.laser.setlidaropt(
            ydlidar.LidarPropSerialPort,
            self.puerto
        )

        self.laser.setlidaropt(
            ydlidar.LidarPropSerialBaudrate,
            128000
        )

        self.laser.setlidaropt(
            ydlidar.LidarPropLidarType,
            ydlidar.TYPE_TRIANGLE
        )

        self.laser.setlidaropt(
            ydlidar.LidarPropDeviceType,
            ydlidar.YDLIDAR_TYPE_SERIAL
        )

        self.laser.setlidaropt(
            ydlidar.LidarPropScanFrequency,
            10.0
        )

        self.laser.setlidaropt(
            ydlidar.LidarPropSampleRate,
            9
        )

        self.laser.setlidaropt(
            ydlidar.LidarPropSingleChannel,
            True
        )

        # =====================================================
        # INICIALIZAR LIDAR
        # =====================================================

        self.get_logger().info(
            'Inicializando YDLIDAR X4...'
        )

        ret = self.laser.initialize()

        if not ret:

            self.get_logger().error(
                'No fue posible inicializar el YDLIDAR X4'
            )

            raise RuntimeError(
                'Error inicializando YDLIDAR'
            )

        self.get_logger().info(
            'YDLIDAR inicializado correctamente'
        )

        # =====================================================
        # ENCENDER LIDAR
        # =====================================================

        ret = self.laser.turnOn()

        if not ret:

            self.get_logger().error(
                'No fue posible encender el YDLIDAR X4'
            )

            raise RuntimeError(
                'Error encendiendo YDLIDAR'
            )

        self.get_logger().info(
            'YDLIDAR encendido correctamente'
        )

        # =====================================================
        # OBJETO PARA LOS BARRIDOS
        # =====================================================

        self.scan = ydlidar.LaserScan()

        # =====================================================
        # TIMER
        # =====================================================

        self.timer = self.create_timer(
            0.05,
            self.leer_lidar
        )

        self.get_logger().info(
            'Midiendo distancia...'
        )

    # =========================================================
    # LECTURA DEL LIDAR
    # =========================================================

    def leer_lidar(self):

        resultado = self.laser.doProcessSimple(
            self.scan
        )

        if not resultado:

            self.get_logger().warning(
                'No se pudo obtener un barrido del LiDAR'
            )

            return

        sumatoria = 0.0
        contador = 0

        # =====================================================
        # CONO ALREDEDOR DE 90 GRADOS
        # =====================================================
        #
        # 1.536 rad ≈ 88 grados
        # 1.606 rad ≈ 92 grados
        #
        # Conservamos la región utilizada en la prueba
        # experimental que ya funcionó.

        for point in self.scan.points:

            if (
                point.angle > 1.536
                and
                point.angle < 1.606
            ):

                if point.angle > math.pi / 2:

                    lx = (
                        point.range
                        *
                        math.cos(
                            point.angle
                            -
                            math.pi / 2
                        )
                    )

                    # Ignorar valores nulos o inválidos

                    if lx > 0:

                        sumatoria += lx
                        contador += 1

        # =====================================================
        # PUBLICAR MEDIDA
        # =====================================================

        if contador > 0:

            promedio = sumatoria / contador

            mensaje = Float32()

            mensaje.data = float(promedio)

            self.publicador_distancia.publish(
                mensaje
            )

            self.get_logger().info(
                f'Distancia: {promedio:.3f} m'
            )

    # =========================================================
    # CIERRE DEL NODO
    # =========================================================

    def destroy_node(self):

        self.get_logger().info(
            'Apagando YDLIDAR...'
        )

        try:

            self.laser.turnOff()
            self.laser.disconnecting()

        except Exception:

            pass

        super().destroy_node()


# =============================================================
# MAIN
# =============================================================

def main(args=None):

    rclpy.init(args=args)

    nodo = None

    try:

        nodo = LidarNode()

        rclpy.spin(nodo)

    except KeyboardInterrupt:

        pass

    except Exception as e:

        print(
            f'Error en lidar_node: {e}'
        )

    finally:

        if nodo is not None:

            nodo.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':

    main()
