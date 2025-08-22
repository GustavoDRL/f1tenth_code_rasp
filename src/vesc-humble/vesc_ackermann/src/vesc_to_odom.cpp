// Copyright 2020 F1TENTH Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
//   * Neither the name of the {copyright_holder} nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// -*- mode:c++; fill-column: 100; -*-

/**
 * @file vesc_to_odom_basic_enhanced.cpp
 * @brief F1TENTH VESC to Odometry - Basic Enhanced Version
 * 
 * Versão otimizada da odometria VESC com melhorias básicas:
 * - Filtros simples de ruído
 * - Validação de dados
 * - Integração numérica melhorada
 * - Covariância adaptativa
 * - Sincronização com servo via tópico real
 * 
 * @author F1TENTH Team
 * @date 2025-01-26
 */

#include "vesc_ackermann/vesc_to_odom.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <string>
#include <deque>
#include <algorithm>
#include <numeric>

namespace vesc_ackermann
{

using geometry_msgs::msg::TransformStamped;
using nav_msgs::msg::Odometry;
using std::placeholders::_1;
using std_msgs::msg::Float64;
using vesc_msgs::msg::VescStateStamped;

/**
 * @class VescToOdom
 * @brief Versão melhorada do VESC to Odometry com filtros básicos
 */
class VescToOdom : public rclcpp::Node
{
private:
  // Frames ROS
  std::string odom_frame_;
  std::string base_frame_;
  
  // Parâmetros de conversão VESC
  double speed_to_erpm_gain_;
  double speed_to_erpm_offset_;
  double steering_to_servo_gain_;
  double steering_to_servo_offset_;
  double wheelbase_actual_;
  bool publish_tf_;
  
  // Parâmetros básicos de qualidade
  double min_speed_threshold_;
  double max_speed_limit_;
  double max_angular_velocity_;
  double max_time_delta_;
  double outlier_threshold_;
  size_t filter_window_size_;
  
  // Parâmetros de covariância
  double base_xy_covariance_;
  double base_yaw_covariance_;
  double speed_covariance_factor_;
  double turn_covariance_factor_;
  
  // Estado da odometria
  double x_, y_, yaw_;
  
  // Buffers para filtros simples
  std::deque<double> speed_buffer_;
  std::deque<double> servo_buffer_;
  std::deque<rclcpp::Time> time_buffer_;
  
  // Último estado válido
  VescStateStamped::SharedPtr last_state_;
  Float64::SharedPtr last_servo_real_;
  
  // Publishers e Subscribers
  rclcpp::Publisher<Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<VescStateStamped>::SharedPtr vesc_state_sub_;
  rclcpp::Subscription<Float64>::SharedPtr servo_real_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
  
  // Estatísticas básicas
  size_t total_updates_;
  double total_distance_;
  size_t outlier_count_;
  rclcpp::Time start_time_;
  size_t log_interval_;
  bool enable_statistics_;
  
public:
  explicit VescToOdom(const rclcpp::NodeOptions & options)
  : Node("vesc_to_odom_node", options),
    odom_frame_("odom"),
    base_frame_("base_link"),
    x_(0.0), y_(0.0), yaw_(0.0),
    total_updates_(0), total_distance_(0.0), outlier_count_(0)
  {
    // Declarar e carregar parâmetros
    declare_and_load_parameters();
    
    // Inicializar buffers
    initialize_buffers();
    
    // Configurar comunicação ROS
    setup_ros_communication();
    
    // Inicializar estatísticas
    start_time_ = now();
    
    RCLCPP_INFO(get_logger(), "🏁 VESC to Odometry Basic Enhanced iniciado");
    RCLCPP_INFO(get_logger(), "📊 Configurações: wheelbase=%.3fm, filtro=%zu samples", 
                wheelbase_actual_, filter_window_size_);
  }
  
private:
  void declare_and_load_parameters()
  {
    // Parâmetros físicos
    declare_parameter("wheelbase_actual", 0.3302);
    declare_parameter("speed_to_erpm_gain", 4614.0);
    declare_parameter("speed_to_erpm_offset", 0.0);
    declare_parameter("steering_to_servo_gain", -1.2135);
    declare_parameter("steering_to_servo_offset", 0.5304);
    
    // Parâmetros de qualidade
    declare_parameter("min_speed_threshold", 0.05);
    declare_parameter("max_speed_limit", 5.0);
    declare_parameter("max_angular_velocity", 3.0);
    declare_parameter("max_time_delta", 0.1);
    declare_parameter("outlier_threshold", 3.0);
    declare_parameter("filter_window_size", 5);
    
    // Parâmetros de covariância
    declare_parameter("base_xy_covariance", 0.01);
    declare_parameter("base_yaw_covariance", 0.05);
    declare_parameter("speed_covariance_factor", 0.1);
    declare_parameter("turn_covariance_factor", 0.5);
    
    // Frames e configuração
    declare_parameter("odom_frame", odom_frame_);
    declare_parameter("base_frame", base_frame_);
    declare_parameter("publish_tf", true);
    
    // Debug e estatísticas
    declare_parameter("enable_statistics", true);
    declare_parameter("log_interval_updates", 1000);
    
    // Carregar valores
    wheelbase_actual_ = get_parameter("wheelbase_actual").as_double();
    speed_to_erpm_gain_ = get_parameter("speed_to_erpm_gain").as_double();
    speed_to_erpm_offset_ = get_parameter("speed_to_erpm_offset").as_double();
    steering_to_servo_gain_ = get_parameter("steering_to_servo_gain").as_double();
    steering_to_servo_offset_ = get_parameter("steering_to_servo_offset").as_double();
    
    min_speed_threshold_ = get_parameter("min_speed_threshold").as_double();
    max_speed_limit_ = get_parameter("max_speed_limit").as_double();
    max_angular_velocity_ = get_parameter("max_angular_velocity").as_double();
    max_time_delta_ = get_parameter("max_time_delta").as_double();
    outlier_threshold_ = get_parameter("outlier_threshold").as_double();
    filter_window_size_ = get_parameter("filter_window_size").as_int();
    
    base_xy_covariance_ = get_parameter("base_xy_covariance").as_double();
    base_yaw_covariance_ = get_parameter("base_yaw_covariance").as_double();
    speed_covariance_factor_ = get_parameter("speed_covariance_factor").as_double();
    turn_covariance_factor_ = get_parameter("turn_covariance_factor").as_double();
    
    odom_frame_ = get_parameter("odom_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    publish_tf_ = get_parameter("publish_tf").as_bool();
    
    enable_statistics_ = get_parameter("enable_statistics").as_bool();
    log_interval_ = get_parameter("log_interval_updates").as_int();
  }
  
  void initialize_buffers()
  {
    // Inicializar buffers com tamanho da janela do filtro
    speed_buffer_.resize(filter_window_size_, 0.0);
    servo_buffer_.resize(filter_window_size_, 0.0);
    time_buffer_.resize(filter_window_size_, now());
  }
  
  void setup_ros_communication()
  {
    // Publisher de odometria
    odom_pub_ = create_publisher<Odometry>("odom", 10);
    
    // TF broadcaster (se habilitado)
    if (publish_tf_) {
      tf_pub_.reset(new tf2_ros::TransformBroadcaster(this));
    }
    
    // Subscriber para estado VESC
    vesc_state_sub_ = create_subscription<VescStateStamped>(
      "sensors/core", 10, std::bind(&VescToOdom::vescStateCallback, this, _1));
    
    // Subscriber para posição real do servo
    servo_real_sub_ = create_subscription<Float64>(
      "/sensors/servo_position_real", 10, 
      std::bind(&VescToOdom::servoRealCallback, this, _1));
  }
  
  void vescStateCallback(const VescStateStamped::SharedPtr state)
  {
    // 1. Validação básica de entrada
    if (!validate_vesc_data(state)) {
      return;
    }
    
    // 2. Conversão de velocidade com validação
    double raw_speed = (state->state.speed - speed_to_erpm_offset_) / speed_to_erpm_gain_;
    
    // Verificar outlier de velocidade
    if (std::abs(raw_speed) > max_speed_limit_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "⚠️ Velocidade fora do limite: %.2f m/s > %.2f m/s", raw_speed, max_speed_limit_);
      outlier_count_++;
      return;
    }
    
    double filtered_speed = apply_moving_average_filter(speed_buffer_, raw_speed);
    
    // 3. Aplicar limiar mínimo de velocidade
    if (std::abs(filtered_speed) < min_speed_threshold_) {
      filtered_speed = 0.0;
    }
    
    // 4. Obter ângulo do servo (filtrado)
    double steering_angle = get_filtered_servo_angle();
    
    // 5. Calcular velocidade angular com limites
    double angular_velocity = 0.0;
    if (std::abs(steering_angle) > 0.001 && std::abs(filtered_speed) > 0.001) {
      angular_velocity = filtered_speed * tan(steering_angle) / wheelbase_actual_;
      
      // Aplicar limite de velocidade angular
      angular_velocity = std::max(-max_angular_velocity_, 
                        std::min(max_angular_velocity_, angular_velocity));
    }
    
    // 6. Integração temporal básica (mas robusta)
    integrate_pose_enhanced(filtered_speed, angular_velocity, state->header.stamp);
    
    // 7. Publicar odometria com covariância apropriada
    publish_enhanced_odometry(state->header.stamp, filtered_speed, angular_velocity);
    
    // 8. Estatísticas simples
    if (enable_statistics_) {
      update_statistics(filtered_speed, angular_velocity);
    }
    
    // Salvar estado para próxima iteração
    last_state_ = state;
  }
  
  void servoRealCallback(const Float64::SharedPtr servo)
  {
    last_servo_real_ = servo;
  }
  
  bool validate_vesc_data(const VescStateStamped::SharedPtr& state)
  {
    // Validação básica dos dados VESC
    if (!state) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "⚠️ Estado VESC nulo");
      return false;
    }
    
    // Verificar se velocidade está em range razoável
    double speed_erpm = state->state.speed;
    if (std::abs(speed_erpm) > 50000) {  // Limite de segurança
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "⚠️ Velocidade VESC fora do range: %.0f ERPM", speed_erpm);
      return false;
    }
    
    // Verificar timestamp válido
    auto current_time = now();
    auto msg_time = rclcpp::Time(state->header.stamp);
    auto time_diff = std::abs((current_time - msg_time).seconds());
    
    if (time_diff > 0.5) {  // Mais de 500ms de diferença
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "⚠️ Timestamp VESC muito antigo: %.1fs", time_diff);
      return false;
    }
    
    return true;
  }
  
  double apply_moving_average_filter(std::deque<double>& buffer, double new_value)
  {
    // Detectar outliers antes de adicionar ao buffer
    if (buffer.size() > 2) {
      double mean = std::accumulate(buffer.begin(), buffer.end(), 0.0) / buffer.size();
      double sq_sum = std::inner_product(buffer.begin(), buffer.end(), buffer.begin(), 0.0);
      double stdev = std::sqrt(sq_sum / buffer.size() - mean * mean);
      
      if (std::abs(new_value - mean) > outlier_threshold_ * stdev) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "⚠️ Outlier detectado: valor=%.3f, média=%.3f, desvio=%.3f", new_value, mean, stdev);
        outlier_count_++;
        // Usar valor anterior ao invés do outlier
        new_value = buffer.back();
      }
    }
    
    // Filtro de média móvel simples
    buffer.pop_front();
    buffer.push_back(new_value);
    
    double sum = std::accumulate(buffer.begin(), buffer.end(), 0.0);
    return sum / buffer.size();
  }
  
  double get_filtered_servo_angle()
  {
    // Usar último ângulo válido do servo
    if (last_servo_real_) {
      double raw_angle = (last_servo_real_->data - steering_to_servo_offset_) / steering_to_servo_gain_;
      return apply_moving_average_filter(servo_buffer_, raw_angle);
    }
    return 0.0;
  }
  
  void integrate_pose_enhanced(double linear_vel, double angular_vel, 
                              const builtin_interfaces::msg::Time& timestamp)
  {
    // Usar último timestamp válido
    if (!last_state_) {
      return;
    }
    
    // Calcular dt com validação
    auto current_time = rclcpp::Time(timestamp);
    auto last_time = rclcpp::Time(last_state_->header.stamp);
    auto dt = (current_time - last_time).seconds();
    
    // Validar dt
    if (dt <= 0.0 || dt > max_time_delta_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "⚠️ Delta tempo inválido: %.3fs", dt);
      return;
    }
    
    // Integração básica com método do ponto médio (mais preciso)
    if (std::abs(angular_vel) < 0.001) {
      // Movimento em linha reta
      x_ += linear_vel * cos(yaw_) * dt;
      y_ += linear_vel * sin(yaw_) * dt;
    } else {
      // Movimento curvilíneo com integração do ponto médio
      double delta_yaw = angular_vel * dt;
      double avg_yaw = yaw_ + delta_yaw * 0.5;
      
      x_ += linear_vel * cos(avg_yaw) * dt;
      y_ += linear_vel * sin(avg_yaw) * dt;
      yaw_ += delta_yaw;
    }
    
    // Normalizar yaw para [-π, π]
    while (yaw_ > M_PI) yaw_ -= 2.0 * M_PI;
    while (yaw_ < -M_PI) yaw_ += 2.0 * M_PI;
  }
  
  void publish_enhanced_odometry(const builtin_interfaces::msg::Time& timestamp,
                                double linear_vel, double angular_vel)
  {
    // Criar mensagem de odometria
    Odometry odom;
    odom.header.stamp = timestamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    
    // Posição
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    
    // Orientação (quaternion usando tf2)
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    
    // Velocidades
    odom.twist.twist.linear.x = linear_vel;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = angular_vel;
    
    // Covariância adaptativa
    set_adaptive_covariance(odom, linear_vel, angular_vel);
    
    // Publicar odometria
    odom_pub_->publish(odom);
    
    // TF (se habilitado)
    if (publish_tf_ && tf_pub_) {
      publish_transform(odom);
    }
  }
  
  void set_adaptive_covariance(Odometry& odom, double linear_vel, double angular_vel)
  {
    // Covariância básica adaptativa baseada em velocidade e curvatura
    double speed_factor = 1.0 + std::abs(linear_vel) * speed_covariance_factor_;
    double turn_factor = 1.0 + std::abs(angular_vel) * turn_covariance_factor_;
    double combined_factor = speed_factor * turn_factor;
    
    // Aplicar fatores adaptativos
    double xy_var = base_xy_covariance_ * combined_factor;
    double yaw_var = base_yaw_covariance_ * turn_factor * 1.5;
    double vel_var = 0.1;                   // 10cm/s base
    double ang_vel_var = 0.1;               // 0.1 rad/s base
    
    // Zerar matriz de covariância
    std::fill(odom.pose.covariance.begin(), odom.pose.covariance.end(), 0.0);
    std::fill(odom.twist.covariance.begin(), odom.twist.covariance.end(), 0.0);
    
    // Aplicar covariâncias
    odom.pose.covariance[0] = xy_var;       // x
    odom.pose.covariance[7] = xy_var;       // y
    odom.pose.covariance[14] = 1000000.0;   // z (não usado)
    odom.pose.covariance[21] = 1000000.0;   // roll (não usado)
    odom.pose.covariance[28] = 1000000.0;   // pitch (não usado)
    odom.pose.covariance[35] = yaw_var;     // yaw
    
    odom.twist.covariance[0] = vel_var;     // vx
    odom.twist.covariance[7] = 1000000.0;   // vy (não usado)
    odom.twist.covariance[14] = 1000000.0;  // vz (não usado)
    odom.twist.covariance[21] = 1000000.0;  // vroll (não usado)
    odom.twist.covariance[28] = 1000000.0;  // vpitch (não usado)
    odom.twist.covariance[35] = ang_vel_var; // vyaw
  }
  
  void publish_transform(const Odometry& odom)
  {
    TransformStamped tf;
    tf.header = odom.header;
    tf.child_frame_id = odom.child_frame_id;
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = odom.pose.pose.orientation;
    
    if (rclcpp::ok()) {
      tf_pub_->sendTransform(tf);
    }
  }
  
  void update_statistics(double linear_vel, double angular_vel)
  {
    total_updates_++;
    total_distance_ += std::abs(linear_vel) * 0.01;  // Aproximação para dt=10ms
    
    // Log estatísticas periodicamente
    if (total_updates_ % log_interval_ == 0) {
      auto elapsed = (now() - start_time_).seconds();
      double actual_freq = total_updates_ / elapsed;
      double outlier_rate = static_cast<double>(outlier_count_) / total_updates_ * 100.0;
      
      RCLCPP_INFO(get_logger(),
        "📊 Odometria: %.1fs, %.2fm percorridos, %.1f Hz, %.1f%% outliers",
        elapsed, total_distance_, actual_freq, outlier_rate);
      
      RCLCPP_INFO(get_logger(),
        "📍 Pose atual: x=%.2fm, y=%.2fm, yaw=%.1f°",
        x_, y_, yaw_ * 180.0 / M_PI);
    }
  }
};

}  // namespace vesc_ackermann

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT

RCLCPP_COMPONENTS_REGISTER_NODE(vesc_ackermann::VescToOdom)