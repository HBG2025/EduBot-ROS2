import math
import statistics

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray

import ydlidar


class LidarNode(Node):

    def __init__(self):

        super().__init__('lidar_node')

        # =====================================================
        # CONFIGURACION DE LOS CONOS
        # =====================================================

        # 360 grados divididos en conos de 10 grados.

        self.apertura_cono = 10.0
        self.paso_conos = 10.0
        self.num_conos = 36

        # Centros:
        #
        # 0, 10, 20, ..., 350 grados

        self.centros = [
            i * self.paso_conos
            for i in range(self.num_conos)
        ]

        # =====================================================
        # RANGO VALIDO DEL LIDAR
        # =====================================================

        self.rango_min = 0.12
        self.rango_max = 10.0

        # =====================================================
        # PUBLICADORES
        # =====================================================

        # Distancia minima de cada uno de los 36 sectores.
        #
        # indice 0  -> centro 0 grados
        # indice 1  -> centro 10 grados
        # indice 2  -> centro 20 grados
        # ...
        # indice 35 -> centro 350 grados

        self.publicador_minimos = self.create_publisher(
            Float32MultiArray,
            '/lidar/sectores_minimos',
            10
        )

        # Mediana de cada sector.
        #
        # Este dato conserva la metodologia utilizada
        # durante el experimento de cobertura.

        self.publicador_medianas = self.create_publisher(
            Float32MultiArray,
            '/lidar/sectores_medianas',
            10
        )

        # =====================================================
        # PUERTO FISICO DEL YDLIDAR
        # =====================================================

        self.puerto = (
            '/dev/serial/by-path/'
            'platform-xhci-hcd.0-usb-0:1:1.0-port0'
        )

        self.get_logger().info(
            f'Puerto YDLIDAR: {self.puerto}'
        )

        # =====================================================
        # INICIALIZAR SDK
        # =====================================================

        ydlidar.os_init()

        self.laser = ydlidar.CYdLidar()

        # =====================================================
        # CONFIGURACION YDLIDAR X4
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
            'Midiendo LiDAR por sectores de 10 grados...'
        )

    # =========================================================
    # NORMALIZAR ANGULO
    # =========================================================

    def normalizar_angulo(self, angulo):

        # Entrada:
        #
        # aproximadamente -180 ... +180
        #
        # Salida:
        #
        # 0 ... 360

        grados = math.degrees(angulo)

        if grados < 0:

            grados += 360.0

        return grados

    # =========================================================
    # OBTENER INDICE DEL CONO
    # =========================================================

    def obtener_indice_cono(self, angulo_grados):

        # Cada cono tiene 10 grados de apertura.
        #
        # Por ejemplo:
        #
        # centro 0°:
        # 355° ... 0° ... 5°
        #
        # centro 10°:
        # 5° ... 10° ... 15°
        #
        # centro 20°:
        # 15° ... 20° ... 25°

        indice = int(
            math.floor(
                (
                    angulo_grados
                    +
                    self.apertura_cono / 2.0
                )
                /
                self.paso_conos
            )
        )

        indice %= self.num_conos

        return indice

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

        # =====================================================
        # LISTAS DE DISTANCIAS
        # =====================================================

        distancias_por_cono = [
            []
            for _ in range(self.num_conos)
        ]

        # =====================================================
        # CLASIFICAR PUNTOS
        # =====================================================

        for point in self.scan.points:

            # -------------------------------------------------
            # Validar rango
            # -------------------------------------------------

            if not (
                self.rango_min
                <=
                point.range
                <=
                self.rango_max
            ):

                continue

            # -------------------------------------------------
            # Convertir angulo
            # -------------------------------------------------

            angulo_grados = self.normalizar_angulo(
                point.angle
            )

            # -------------------------------------------------
            # Determinar cono
            # -------------------------------------------------

            indice = self.obtener_indice_cono(
                angulo_grados
            )

            # -------------------------------------------------
            # Guardar distancia
            # -------------------------------------------------

            distancias_por_cono[indice].append(
                point.range
            )

        # =====================================================
        # CALCULAR MINIMOS Y MEDIANAS
        # =====================================================

        distancias_minimas = []
        distancias_medianas = []

        for i in range(self.num_conos):

            datos = distancias_por_cono[i]

            if datos:

                distancia_minima = min(
                    datos
                )

                distancia_mediana = statistics.median(
                    datos
                )

            else:

                # NaN indica que ese sector no tuvo
                # una medicion valida en este barrido.

                distancia_minima = float('nan')
                distancia_mediana = float('nan')

            distancias_minimas.append(
                float(distancia_minima)
            )

            distancias_medianas.append(
                float(distancia_mediana)
            )

        # =====================================================
        # PUBLICAR MINIMOS
        # =====================================================

        mensaje_minimos = Float32MultiArray()

        mensaje_minimos.data = (
            distancias_minimas
        )

        self.publicador_minimos.publish(
            mensaje_minimos
        )

        # =====================================================
        # PUBLICAR MEDIANAS
        # =====================================================

        mensaje_medianas = Float32MultiArray()

        mensaje_medianas.data = (
            distancias_medianas
        )

        self.publicador_medianas.publish(
            mensaje_medianas
        )

        # =====================================================
        # INFORMACION DE DEPURACION
        # =====================================================

        # Mostrar algunos sectores para poder verificar
        # rapidamente el funcionamiento.

        sectores_mostrar = [
            0,
            10,
            20,
            30,
            40,
            50,
            60,
            70,
            80,
            90,
            100,
            110,
            120,
            130,
            140,
            150,
            160,
            170
        ]

        texto = []

        for angulo in sectores_mostrar:

            indice = int(
                angulo / 10
            )

            valor = distancias_minimas[indice]

            if math.isnan(valor):

                texto.append(
                    f'{angulo}°: --'
                )

            else:

                texto.append(
                    f'{angulo}°: {valor:.2f}m'
                )

        self.get_logger().info(
            ' | '.join(texto)
        )

    # =========================================================
    # CIERRE
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
