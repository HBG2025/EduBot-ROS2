import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from std_msgs.msg import String


class ObstacleAvoidance(Node):

    def __init__(self):

        super().__init__('obstacle_avoidance')

        # =====================================================
        # PARÁMETROS DE EVASIÓN
        # =====================================================

        # Obstáculo detectado a 15 cm
        self.distancia_obstaculo = 0.15

        # Después de detenerse, debe haber al menos 20 cm
        # para considerar nuevamente el camino libre.
        self.distancia_libre = 0.20

        # Giro utilizado para intentar evadir el obstáculo
        self.angulo_giro = 45

        # Tiempo estimado para permitir que termine el giro.
        # Nuestro comando "girar XX" todavía es bloqueante
        # dentro del ESP32.
        self.tiempo_espera_giro = 3.0

        # =====================================================
        # VARIABLES DE ESTADO
        # =====================================================

        self.estado = 'INICIO'

        self.distancia_actual = None

        self.tiempo_fin_giro = 0.0

        # =====================================================
        # PUBLICADOR DE COMANDOS
        # =====================================================

        self.publicador_comando = self.create_publisher(
            String,
            '/robot/comando',
            10
        )

        # =====================================================
        # SUSCRIPTOR A LA DISTANCIA DEL LIDAR
        # =====================================================

        self.suscriptor_lidar = self.create_subscription(
            Float32,
            '/lidar/distancia',
            self.recibir_distancia,
            10
        )

        # =====================================================
        # TIMER DE CONTROL
        # =====================================================

        self.timer = self.create_timer(
            0.10,
            self.control
        )

        self.get_logger().info(
            '======================================'
        )

        self.get_logger().info(
            ' EVASION DE OBSTACULOS INICIADA'
        )

        self.get_logger().info(
            '======================================'
        )

        self.get_logger().info(
            'Obstaculo <= 0.15 m'
        )

        self.get_logger().info(
            'Camino libre >= 0.20 m'
        )

    # =========================================================
    # RECIBIR DISTANCIA DEL LIDAR
    # =========================================================

    def recibir_distancia(self, msg):

        self.distancia_actual = msg.data

    # =========================================================
    # ENVIAR COMANDO AL ESP32
    # =========================================================

    def enviar_comando(self, comando):

        mensaje = String()

        mensaje.data = comando

        self.publicador_comando.publish(
            mensaje
        )

        self.get_logger().info(
            f'COMANDO -> {comando}'
        )

    # =========================================================
    # CONTROL DE EVASIÓN
    # =========================================================

    def control(self):

        # -----------------------------------------------------
        # Esperar hasta tener una medida del LiDAR
        # -----------------------------------------------------

        if self.distancia_actual is None:

            return

        tiempo_actual = (
            self.get_clock().now().nanoseconds
            / 1e9
        )

        # =====================================================
        # ESTADO INICIAL
        # =====================================================

        if self.estado == 'INICIO':

            self.get_logger().info(
                f'Distancia inicial: '
                f'{self.distancia_actual:.3f} m'
            )

            if (
                self.distancia_actual
                >
                self.distancia_obstaculo
            ):

                self.enviar_comando(
                    'adelante'
                )

                self.estado = 'AVANZANDO'

            else:

                self.enviar_comando(
                    'parar'
                )

                self.estado = 'OBSTACULO'

        # =====================================================
        # ROBOT AVANZANDO
        # =====================================================

        elif self.estado == 'AVANZANDO':

            if (
                self.distancia_actual
                <=
                self.distancia_obstaculo
            ):

                self.get_logger().warning(
                    '=============================='
                )

                self.get_logger().warning(
                    f'OBSTACULO A '
                    f'{self.distancia_actual:.3f} m'
                )

                self.get_logger().warning(
                    '=============================='
                )

                self.enviar_comando(
                    'parar'
                )

                self.estado = 'OBSTACULO'

        # =====================================================
        # OBSTÁCULO DETECTADO
        # =====================================================

        elif self.estado == 'OBSTACULO':

            self.get_logger().info(
                'Iniciando maniobra de evasión'
            )

            comando = (
                f'girar {self.angulo_giro}'
            )

            self.enviar_comando(
                comando
            )

            self.tiempo_fin_giro = (
                tiempo_actual
                +
                self.tiempo_espera_giro
            )

            self.estado = 'GIRANDO'

        # =====================================================
        # ESPERANDO QUE TERMINE EL GIRO
        # =====================================================

        elif self.estado == 'GIRANDO':

            if (
                tiempo_actual
                >=
                self.tiempo_fin_giro
            ):

                self.estado = 'VERIFICAR'

        # =====================================================
        # VERIFICAR CAMINO DESPUÉS DEL GIRO
        # =====================================================

        elif self.estado == 'VERIFICAR':

            self.get_logger().info(
                f'Distancia despues del giro: '
                f'{self.distancia_actual:.3f} m'
            )

            if (
                self.distancia_actual
                >=
                self.distancia_libre
            ):

                self.get_logger().info(
                    'Camino libre'
                )

                self.enviar_comando(
                    'adelante'
                )

                self.estado = 'AVANZANDO'

            else:

                self.get_logger().warning(
                    'El obstaculo continua'
                )

                self.get_logger().info(
                    'Se realizara otro giro'
                )

                self.estado = 'OBSTACULO'


# =============================================================
# MAIN
# =============================================================

def main(args=None):

    rclpy.init(args=args)

    nodo = ObstacleAvoidance()

    try:

        rclpy.spin(nodo)

    except KeyboardInterrupt:

        pass

    # ---------------------------------------------------------
    # PARADA DE SEGURIDAD AL CERRAR EL NODO
    # ---------------------------------------------------------

    nodo.enviar_comando(
        'parar'
    )

    nodo.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':

    main()
