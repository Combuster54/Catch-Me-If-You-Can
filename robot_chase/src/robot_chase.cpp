#include <chrono>
#include <geometry_msgs/msg/twist.hpp>

#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/transform_listener.h>

class RobotChaseNode : public rclcpp::Node {

public:
  int count = 0;
  RobotChaseNode()
      : Node("robot_chase"), tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_) {

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("rick/cmd_vel", 10);

    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(200),
                                std::bind(&RobotChaseNode::sendCommand, this));
  }

  void sendCommand() {
    try {

      //   RCLCPP_INFO(this->get_logger(), "HELLO");

      // Obtener la transformación entre "rick/base_link" y "morty/base_link"
      auto transform = tf_buffer_.lookupTransform(
          "rick/base_link", "morty/base_link", tf2::TimePoint(),
          tf2::Duration(static_cast<long int>(0.1 * 1e9)));

      // Calcular la distancia y el error angular entre los frames de referencia
      double error_distance = sqrt(pow(transform.transform.translation.x, 2) +
                                   pow(transform.transform.translation.y, 2));
      double error_yaw = atan2(transform.transform.translation.y,
                               transform.transform.translation.x);

    //   RCLCPP_INFO(this->get_logger(), "error_distance : %f", error_distance);
    //   RCLCPP_INFO(this->get_logger(), "error_yaw : %f", error_yaw);

      // Definir la velocidad angular y lineal
      double kp_yaw =
          0.5; // Valor fijo para el controlador de la velocidad angular
      double kp_distance =
          0.2; // Valor fijo para el controlador de la velocidad lineal
      double angular_vel = kp_yaw * error_yaw;
      double linear_vel = kp_distance * error_distance;

    //   RCLCPP_INFO(this->get_logger(), "kp_yaw : %f", kp_yaw);
    //   RCLCPP_INFO(this->get_logger(), "kp_distance : %f", kp_distance);
    //   RCLCPP_INFO(this->get_logger(), "angular_vel : %f", angular_vel);
    //   RCLCPP_INFO(this->get_logger(), "linear_vel : %f", linear_vel);

      // Menor a esto es que Rick ya alcanzo a Morty y ya colisionaron, asi que
      // no hay razon para seguir avanzando
      if (error_distance <= 0.38) {

        // count +=1;
        RCLCPP_INFO(this->get_logger(), "Rick caught Morty");
        this->cmd_vel.angular.z = 0;
        this->cmd_vel.linear.x = 0;
        publisher_->publish(this->cmd_vel);
      } else {
        // Sigue a Morty
        this->cmd_vel.angular.z = angular_vel;
        this->cmd_vel.linear.x = linear_vel;
        publisher_->publish(this->cmd_vel);
      }
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    }
  }

private:
  geometry_msgs::msg::Twist cmd_vel;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  tf2::TimePoint tf_time_zero = tf2::TimePoint(std::chrono::seconds(0));
  double kp_distance_;
  double kp_yaw_;
};

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  auto robotChase = std::make_shared<RobotChaseNode>();
  rclcpp::spin(robotChase);
  rclcpp::shutdown();

  return 0;
}