#!/usr/bin/env python3
"""
Servo Feedback Publisher para F1TENTH
=====================================
Publica posição real estimada do servo para odometria básica otimizada.

Este nó estima a posição real do servo baseado no modelo dinâmico
e publica no tópico /sensors/servo_position_real para ser usado
pela odometria melhorada.

Author: F1TENTH Team
Date: 2025-01-26
"""

import time
import collections
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64


class ServoFeedbackPublisher(Node):
    """
    Publisher de feedback do servo para odometria básica.
    
    Estima a posição real do servo baseado em modelo dinâmico simples
    e publica para sincronização com a odometria VESC.
    """
    
    def __init__(self):
        super().__init__('servo_feedback_publisher')
        
        # Parâmetros do modelo dinâmico do servo
        self.declare_parameter('servo_time_constant', 0.05)  # 50ms resposta típica
        self.declare_parameter('feedback_frequency', 100.0)  # 100Hz
        self.declare_parameter('max_servo_rate', 10.0)       # 10 rad/s máximo
        
        self.servo_time_constant = self.get_parameter('servo_time_constant').value
        self.feedback_frequency = self.get_parameter('feedback_frequency').value
        self.max_servo_rate = self.get_parameter('max_servo_rate').value
        
        # Estado do modelo dinâmico
        self.actual_angle = 0.0
        self.target_angle = 0.0
        self.last_update_time = time.time()
        self.angle_lock = threading.RLock()
        
        # Histórico para suavização
        self.angle_history = collections.deque(maxlen=10)
        
        # QoS otimizado para baixa latência
        self.low_latency_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publisher para posição real estimada
        self.feedback_pub = self.create_publisher(
            Float64, 
            '/sensors/servo_position_real', 
            self.low_latency_qos
        )
        
        # Subscriber para comandos do servo
        self.command_sub = self.create_subscription(
            Float64,
            '/sensors/servo_position_command',  # Tópico interno do enhanced_servo_control
            self.command_callback,
            self.low_latency_qos
        )
        
        # Timer de alta frequência para feedback
        timer_period = 1.0 / self.feedback_frequency
        self.feedback_timer = self.create_timer(timer_period, self.publish_feedback)
        
        # Estatísticas
        self.feedback_count = 0
        self.start_time = time.time()
        
        self.get_logger().info(
            f"🔄 Servo Feedback Publisher iniciado - {self.feedback_frequency}Hz"
        )
    
    def command_callback(self, msg: Float64):
        """Recebe comando de ângulo do servo."""
        with self.angle_lock:
            self.target_angle = msg.data
    
    def publish_feedback(self):
        """Publica estimativa da posição real do servo."""
        current_time = time.time()
        
        with self.angle_lock:
            # Calcular dt
            dt = current_time - self.last_update_time
            if dt <= 0.0:
                return
            
            # Modelo de primeira ordem para resposta do servo
            # servo_response = target * (1 - exp(-t/tau))
            alpha = dt / (self.servo_time_constant + dt)
            
            # Atualizar ângulo atual (modelo dinâmico)
            angle_error = self.target_angle - self.actual_angle
            
            # Aplicar limite de velocidade angular
            max_change = self.max_servo_rate * dt
            if abs(angle_error) > max_change:
                angle_error = max_change if angle_error > 0 else -max_change
            
            self.actual_angle += alpha * angle_error
            
            # Adicionar ao histórico para suavização
            self.angle_history.append(self.actual_angle)
            
            # Aplicar filtro de média móvel
            if len(self.angle_history) > 1:
                smoothed_angle = sum(self.angle_history) / len(self.angle_history)
            else:
                smoothed_angle = self.actual_angle
            
            self.last_update_time = current_time
        
        # Publicar posição real estimada
        feedback_msg = Float64()
        feedback_msg.data = smoothed_angle
        self.feedback_pub.publish(feedback_msg)
        
        # Estatísticas
        self.feedback_count += 1
        if self.feedback_count % 1000 == 0:  # Log a cada 10s @ 100Hz
            elapsed = current_time - self.start_time
            actual_freq = self.feedback_count / elapsed
            self.get_logger().info(
                f"📊 Servo Feedback: {actual_freq:.1f}Hz, ângulo atual: {smoothed_angle:.3f}rad"
            )
    
    def get_current_angle(self) -> float:
        """Retorna o ângulo atual estimado (thread-safe)."""
        with self.angle_lock:
            return self.actual_angle


def main(args=None):
    """Entry point para o servo feedback publisher."""
    rclpy.init(args=args)
    
    try:
        node = ServoFeedbackPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()