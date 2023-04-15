#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

class BotOdomToTf : public rclcpp::Node
{
public:
  explicit BotOdomToTf(const std::string& robot_base_frame = "/rick/odom")
    : Node("rick_tf"), robot_base_frame_{robot_base_frame}
  {
    init_tf_message();

    subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
      "/rick/odom", rclcpp::QoS{1}.durability_volatile().best_effort(),
      std::bind(&BotOdomToTf::listener_callback, this, std::placeholders::_1));

    br_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    RCLCPP_WARN(get_logger(), "world_to_rick READY!");
  }

private:
  void listener_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    update_data(msg);
    RCLCPP_DEBUG(get_logger(), "Odom VALUE: '%f'", rick_odom.pose.pose.position.x);
    broadcast_new_tf();
  }

  void update_data(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    rick_odom = *msg;
  }

  std::tuple<geometry_msgs::msg::Point, geometry_msgs::msg::Quaternion, rclcpp::Time>
  get_odom_data() const
  {
    const auto time_header = rick_odom.header.stamp;
    const auto position = rick_odom.pose.pose.position;
    const auto orientation = rick_odom.pose.pose.orientation;
    return  std::make_tuple(position, orientation,time_header);

  }

  void init_tf_message()
  {
    transform_stamped_.header.frame_id = "world";
    transform_stamped_.child_frame_id = robot_base_frame_;
  }

  void broadcast_new_tf()
  {
    const auto [position, orientation, time_header] = get_odom_data();

    transform_stamped_.header.stamp = time_header;
    transform_stamped_.transform.translation.x = position.x;
    transform_stamped_.transform.translation.y = position.y;
    transform_stamped_.transform.translation.z = position.z;
    transform_stamped_.transform.rotation.x = orientation.x;
    transform_stamped_.transform.rotation.y = orientation.y;
    transform_stamped_.transform.rotation.z = orientation.z;
    transform_stamped_.transform.rotation.w = orientation.w;

    br_->sendTransform(transform_stamped_);
  }

  std::string robot_base_frame_;
  nav_msgs::msg::Odometry rick_odom;
  geometry_msgs::msg::TransformStamped transform_stamped_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> br_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto odom_to_tf_obj = std::make_shared<BotOdomToTf>();
  rclcpp::spin(odom_to_tf_obj);
  rclcpp::shutdown();
  return 0;
}
