import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String


class ObstacleAvoidance(Node):

    def __init__(self):

        super().__init__('obstacle_avoidance')

        # =====================================================
        # CONFIGURACION DE EVASION
        # =====================================================

        # Angulo que experimentalmente corresponde
        # al frente fisico del robot.

        self.angulo_frente = 180

        # -----------------------------------------------------
        # Sectores utilizados
        # -----------------------------------------------------

        # Frente:
        # 160, 170, 180, 190, 200

        self.sectores_frente = [
            160,
            170,
            180,
            190,
            200
        ]

        # Izquierda:
        # 110, 120, 130, 140, 150

        self.sectores_izquierda = [
            110,
            120,
            130,
            140,
            150
        ]

        # Derecha:
        # 210, 220, 230, 240, 250

        self.sectores_derecha = [
            210,
            220,
            230,
            240,
            250
        ]

        # -----------------------------------------------------
        # Distancias
        # -----------------------------------------------------

        self.distancia_obstaculo = 0.30

        self.distancia_libre = 0.40

        # -----------------------------------------------------
        # Giro
        # -----------------------------------------------------

        self.angulo_giro = 45

        # Tiempo estimado para el giro del robot.

        self.tiempo_espera_giro = 2.0

        # =====================================================
        # ESTADO
        # =====================================================

        self.estado = 'ESPERANDO_LIDAR'

        self.sectores_minimos = None

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
        # SUSCRIPTOR DE SECTORES DEL LIDAR
        # =====================================================

        self.suscriptor_lidar = self.create_subscription(
            Float32MultiArray,
            '/lidar/sectores_minimos',
            self.recibir_sectores,
            10
        )

        # =====================================================
        # TIMER DE CONTROL
        # =====================================================

        self.timer = self.create_timer(
            0.05,
            self.control
        )

        # =====================================================
        # MENSAJES DE INICIO
        # =====================================================

        self.get_logger().info(
            '======================================'
        )

        self.get_logger().info(
            ' EVASION DE OBSTACULOS POR SECTORES'
        )

        self.get_logger().info(
            '======================================'
        )

        self.get_logger().info(
            f'Frente fisico: {self.angulo_frente} grados'
        )

        self.get_logger().info(
            'Obstaculo <= 0.30 m'
        )

        self.get_logger().info(
            'Camino libre >= 0.40 m'
        )

        self.get_logger().info(
            f'Giro configurado: {self.angulo_giro} grados'
        )

        self.get_logger().info(
            f'Tiempo estimado de giro: '
            f'{self.tiempo_espera_giro:.1f} s'
        )

    # =========================================================
    # RECIBIR SECTORES DEL LIDAR
    # =========================================================

    def recibir_sectores(self, msg):

        if len(msg.data) != 36:

            self.get_logger().warning(
                f'Se recibieron {len(msg.data)} sectores '
                f'en lugar de 36'
            )

            return

        self.sectores_minimos = list(
            msg.data
        )

    # =========================================================
    # OBTENER DISTANCIA DE UN SECTOR
    # =========================================================

    def obtener_distancia_sector(self, angulo):

        indice = int(
            angulo / 10
        )

        if indice < 0 or indice >= 36:

            return float('nan')

        distancia = self.sectores_minimos[indice]

        if math.isnan(distancia):

            return float('nan')

        return float(distancia)

    # =========================================================
    # OBTENER DISTANCIA DE UNA ZONA
    # =========================================================

    def obtener_distancia_zona(self, sectores):

        valores = []

        for angulo in sectores:

            distancia = self.obtener_distancia_sector(
                angulo
            )

            if not math.isnan(distancia):

                valores.append(
                    distancia
                )

        if not valores:

            return float('nan')

        # Para seguridad utilizamos la distancia minima
        # de la zona.

        return min(valores)

    # =========================================================
    # ENVIAR COMANDO
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
    # MOSTRAR ESTADO DE LAS ZONAS
    # =========================================================

    def mostrar_zonas(self):

        frente = self.obtener_distancia_zona(
            self.sectores_frente
        )

        izquierda = self.obtener_distancia_zona(
            self.sectores_izquierda
        )

        derecha = self.obtener_distancia_zona(
            self.sectores_derecha
        )

        self.get_logger().info(
            f'ZONAS -> '
            f'Frente: {frente:.3f} m | '
            f'Izquierda: {izquierda:.3f} m | '
            f'Derecha: {derecha:.3f} m'
        )

        return (
            frente,
            izquierda,
            derecha
        )

    # =========================================================
    # CONTROL PRINCIPAL
    # =========================================================

    def control(self):

        # -----------------------------------------------------
        # Todavia no tenemos datos del LiDAR
        # -----------------------------------------------------

        if self.sectores_minimos is None:

            return

        tiempo_actual = (
            self.get_clock().now().nanoseconds
            / 1e9
        )

        # =====================================================
        # ESPERANDO DATOS INICIALES
        # =====================================================

        if self.estado == 'ESPERANDO_LIDAR':

            self.get_logger().info(
                'Datos de sectores recibidos.'
            )

            self.estado = 'INICIO'

        # =====================================================
        # ESTADO INICIAL
        # =====================================================

        elif self.estado == 'INICIO':

            frente = self.obtener_distancia_zona(
                self.sectores_frente
            )

            izquierda = self.obtener_distancia_zona(
                self.sectores_izquierda
            )

            derecha = self.obtener_distancia_zona(
                self.sectores_derecha
            )

            self.get_logger().info(
                f'INICIO -> '
                f'Frente: {frente:.3f} m | '
                f'Izquierda: {izquierda:.3f} m | '
                f'Derecha: {derecha:.3f} m'
            )

            if math.isnan(frente):

                self.get_logger().warning(
                    'Sin medicion valida al frente'
                )

                return

            # -------------------------------------------------
            # Camino libre
            # -------------------------------------------------

            if frente >= self.distancia_libre:

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

            frente = self.obtener_distancia_zona(
                self.sectores_frente
            )

            if math.isnan(frente):

                self.get_logger().warning(
                    'No hay medicion valida del frente'
                )

                return

            # -------------------------------------------------
            # Obstaculo detectado
            # -------------------------------------------------

            if frente <= self.distancia_obstaculo:

                self.get_logger().warning(
                    '=============================='
                )

                self.get_logger().warning(
                    f'OBSTACULO AL FRENTE: '
                    f'{frente:.3f} m'
                )

                self.get_logger().warning(
                    '=============================='
                )

                self.enviar_comando(
                    'parar'
                )

                self.estado = 'OBSTACULO'

        # =====================================================
        # OBSTACULO DETECTADO
        # =====================================================

        elif self.estado == 'OBSTACULO':

            izquierda = self.obtener_distancia_zona(
                self.sectores_izquierda
            )

            derecha = self.obtener_distancia_zona(
                self.sectores_derecha
            )

            self.get_logger().info(
                f'EVALUANDO EVASION -> '
                f'Izquierda: {izquierda:.3f} m | '
                f'Derecha: {derecha:.3f} m'
            )

            # -------------------------------------------------
            # Ninguno de los lados tiene medicion
            # -------------------------------------------------

            if (
                math.isnan(izquierda)
                and
                math.isnan(derecha)
            ):

                self.get_logger().warning(
                    'No hay mediciones validas '
                    'a izquierda ni derecha'
                )

                return

            # -------------------------------------------------
            # Solo izquierda disponible
            # -------------------------------------------------

            if math.isnan(derecha):

                lado = 'izquierda'

            # -------------------------------------------------
            # Solo derecha disponible
            # -------------------------------------------------

            elif math.isnan(izquierda):

                lado = 'derecha'

            # -------------------------------------------------
            # Ambas disponibles
            # -------------------------------------------------

            elif izquierda >= derecha:

                lado = 'izquierda'

            else:

                lado = 'derecha'

            # -------------------------------------------------
            # Ejecutar giro
            # -------------------------------------------------

            if lado == 'izquierda':

                comando = (
                    f'girar -{self.angulo_giro}'
                )

                self.get_logger().info(
                    'Espacio mayor a la IZQUIERDA'
                )

            else:

                comando = (
                    f'girar {self.angulo_giro}'
                )

                self.get_logger().info(
                    'Espacio mayor a la DERECHA'
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
        # ESPERANDO FIN DEL GIRO
        # =====================================================

        elif self.estado == 'GIRANDO':

            if (
                tiempo_actual
                >=
                self.tiempo_fin_giro
            ):

                self.estado = 'VERIFICAR'

        # =====================================================
        # VERIFICAR DESPUES DEL GIRO
        # =====================================================

        elif self.estado == 'VERIFICAR':

            frente = self.obtener_distancia_zona(
                self.sectores_frente
            )

            izquierda = self.obtener_distancia_zona(
                self.sectores_izquierda
            )

            derecha = self.obtener_distancia_zona(
                self.sectores_derecha
            )

            self.get_logger().info(
                f'VERIFICACION -> '
                f'Frente: {frente:.3f} m | '
                f'Izquierda: {izquierda:.3f} m | '
                f'Derecha: {derecha:.3f} m'
            )

            if (
                not math.isnan(frente)
                and
                frente >= self.distancia_libre
            ):

                self.get_logger().info(
                    'Camino libre despues del giro'
                )

                self.enviar_comando(
                    'adelante'
                )

                self.estado = 'AVANZANDO'

            else:

                self.get_logger().warning(
                    'El obstaculo continua'
                )

                self.enviar_comando(
                    'parar'
                )

                self.estado = 'OBSTACULO'

    # =========================================================
    # CIERRE
    # =========================================================

    def destroy_node(self):

        # Intentar detener el robot antes de cerrar.

        try:

            self.enviar_comando(
                'parar'
            )

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

        nodo = ObstacleAvoidance()

        rclpy.spin(nodo)

    except KeyboardInterrupt:

        pass

    except Exception as e:

        print(
            f'Error en obstacle_avoidance: {e}'
        )

    finally:

        if nodo is not None:

            nodo.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':

    main()
