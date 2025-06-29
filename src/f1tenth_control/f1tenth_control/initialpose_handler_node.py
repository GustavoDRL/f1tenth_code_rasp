#!/usr/bin/env python3
"""
Initial Pose Handler Node
=========================
Trata o tópico /initialpose para compatibilidade com simulador F1TENTH.

Este nó permite que códigos testados no simulador F1TENTH funcionem
corretamente no hardware real, especialmente aqueles que dependem
do reset de pose via RViz usando a ferramenta "2D Pose Estimate".

Funcionalidades:
- Subscreve ao tópico /initialpose
- Redefine a pose do robô no sistema de coordenadas
- Reseta odometria quando necessário
- Publica transformações atualizadas

Autor: F1TENTH Team
Data: 2025-06-28
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math


class InitialPoseHandler(Node):
    """
    Node para tratar o tópico /initialpose.

    Características:
    - Compatibilidade total com RViz 2D Pose Estimate
    - Reset de odometria baseado na nova pose
    - Atualização de transformações TF
    - Logging detalhado para debug
    """

    def __init__(self):
        super().__init__("initialpose_handler_node")

        # Declarar parâmetros
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("publish_tf", True)

        # Obter parâmetros
        self.base_frame = self.get_parameter("base_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.map_frame = self.get_parameter("map_frame").value
        self.publish_tf = self.get_parameter("publish_tf").value

        # Subscriber para /initialpose
        self.initialpose_subscription = self.create_subscription(
            PoseWithCovarianceStamped, "initialpose", self.initialpose_callback, 10
        )

        # Publisher para odometria resetada (opcional)
        self.odom_publisher = self.create_publisher(Odometry, "odom_reset", 10)

        # TF Broadcaster
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        # Estado interno
        self.current_pose = None
        self.last_reset_time = self.get_clock().now()

        self.get_logger().info("Initial Pose Handler initialized")
        self.get_logger().info(
            f"Frames: map={self.map_frame}, "
            f"odom={self.odom_frame}, base={self.base_frame}"
        )

    def initialpose_callback(self, msg: PoseWithCovarianceStamped):
        """
        Callback para processar mensagens /initialpose.

        Args:
            msg: Mensagem de pose inicial recebida do RViz
        """
        self.get_logger().info("Received initial pose from RViz")

        # Armazenar a nova pose
        self.current_pose = msg

        # Log da pose recebida
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        self.get_logger().info(
            f"New pose: x={position.x:.3f}, y={position.y:.3f}, " f"z={position.z:.3f}"
        )

        # Converter quaternion para ângulo yaw para logging
        # Usar fórmula simplificada para yaw a partir de quaternion
        yaw_rad = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        yaw_degrees = yaw_rad * 180.0 / math.pi

        self.get_logger().info(f"Orientation: yaw={yaw_degrees:.1f}°")

        # Processar reset da pose
        self.process_pose_reset(msg)

        # Atualizar timestamp do último reset
        self.last_reset_time = self.get_clock().now()

    def process_pose_reset(self, pose_msg: PoseWithCovarianceStamped):
        """
        Processa o reset da pose do robô.

        Args:
            pose_msg: Nova pose do robô
        """
        # Publicar odometria resetada
        self.publish_reset_odometry(pose_msg)

        # Publicar transformação atualizada (se habilitado)
        if self.publish_tf:
            self.publish_map_to_odom_transform(pose_msg)

    def publish_reset_odometry(self, pose_msg: PoseWithCovarianceStamped):
        """
        Publica mensagem de odometria resetada.

        Args:
            pose_msg: Nova pose base
        """
        odom_msg = Odometry()

        # Header
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        # Pose (copiar da mensagem recebida)
        odom_msg.pose = pose_msg.pose

        # Velocidade zerada (robô parado após reset)
        odom_msg.twist.twist.linear.x = 0.0
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0

        # Covariância da velocidade
        odom_msg.twist.covariance[0] = 0.1  # vx
        odom_msg.twist.covariance[7] = 0.1  # vy
        odom_msg.twist.covariance[35] = 0.1  # vyaw

        # Publicar
        self.odom_publisher.publish(odom_msg)

        self.get_logger().info("Published reset odometry")

    def publish_map_to_odom_transform(self, pose_msg: PoseWithCovarianceStamped):
        """
        Publica transformação map->odom baseada na nova pose.

        Args:
            pose_msg: Nova pose para calcular a transformação
        """
        if not self.publish_tf:
            return

        transform = TransformStamped()

        # Header
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.odom_frame

        # Para simplificar, assumimos que a pose inicial é a origem
        # Em uma implementação mais avançada, isso seria calculado
        # baseado na diferença entre a pose atual e a nova pose
        transform.transform.translation.x = pose_msg.pose.pose.position.x
        transform.transform.translation.y = pose_msg.pose.pose.position.y
        transform.transform.translation.z = pose_msg.pose.pose.position.z

        transform.transform.rotation = pose_msg.pose.pose.orientation

        # Publicar transformação
        self.tf_broadcaster.sendTransform(transform)

        self.get_logger().info("Published map->odom transform")

    def get_pose_status(self) -> dict:
        """
        Retorna status atual da pose.

        Returns:
            dict: Informações sobre o estado da pose
        """
        if self.current_pose is None:
            return {"status": "no_pose_set"}

        time_since_reset = (
            self.get_clock().now() - self.last_reset_time
        ).nanoseconds / 1e9

        return {
            "status": "pose_set",
            "time_since_reset": time_since_reset,
            "last_pose": self.current_pose,
        }


def main(args=None):
    """Função principal."""
    rclpy.init(args=args)

    node = InitialPoseHandler()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down initialpose_handler_node")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
