#include "nav_msgs/msg/odometry.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class RobotFollower : public rclcpp::Node {
public:
  RobotFollower()
      : Node("robot_follower"), tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_) {
    {
      cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
          "/rick/cmd_vel", 10);

      odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
          "/morty/odom", 10,
          std::bind(&RobotFollower::odomCallback, this, std::placeholders::_1));
    }
  }

  void startFollowing() {
    rclcpp::Rate rate(10); // 10 Hz

    while (rclcpp::ok()) {
      try {
        // Obtener la transformada actual de world a rick/odom
        geometry_msgs::msg::TransformStamped transform_stamped =
            tf_buffer_.lookupTransform("world", "rick/odom", rclcpp::Time(0));
        // Calcular la distancia y dirección de Morty en relación con la
        // posición actual de Rick
        double distance =
            sqrt(pow(morty_pose_.position.x -
                         transform_stamped.transform.translation.x,
                     2) +
                 pow(morty_pose_.position.y -
                         transform_stamped.transform.translation.y,
                     2));
        double direction = atan2(
            morty_pose_.position.y - transform_stamped.transform.translation.y,
            morty_pose_.position.x - transform_stamped.transform.translation.x);
        // Si la distancia es menor que la distancia de seguimiento, detener a
        // Rick
        if (distance < follow_distance_) {
          geometry_msgs::msg::Twist cmd_vel;
          cmd_vel.linear.x = 0.0;
          cmd_vel.angular.z = 0.0;
          cmd_vel_pub_->publish(cmd_vel);
        }
        // De lo contrario, seguir a Morty
        else {
          geometry_msgs::msg::Twist cmd_vel;
          cmd_vel.linear.x = follow_linear_velocity_;
          cmd_vel.angular.z = follow_angular_velocity_ * direction;
          cmd_vel_pub_->publish(cmd_vel);
        }
      } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
      }
      rclcpp::spin_some(shared_from_this());
      rate.sleep();
    }
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    morty_pose_ = msg->pose.pose;
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  geometry_msgs::msg::Pose morty_pose_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  double follow_distance_ = 1.0; // Distancia de seguimiento en metros
  double follow_linear_velocity_ =
      0.5; // Velocidad lineal constante para seguir a Morty
  double follow_angular_velocity_ =
      1.0; // Velocidad angular para girar hacia Morty
};

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);

  auto node = std::make_shared<RobotFollower>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}