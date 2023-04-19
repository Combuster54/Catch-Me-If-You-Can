#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.h>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("static_tf_publisher");

    tf2_ros::StaticTransformBroadcaster broadcaster(node);

    geometry_msgs::msg::TransformStamped static_transform_morty;
    static_transform_morty.header.stamp = node->now();
    static_transform_morty.header.frame_id = "world";
    static_transform_morty.child_frame_id = "morty/odom";
    static_transform_morty.transform.translation.x = 0.0;
    static_transform_morty.transform.translation.y = 0.0;
    static_transform_morty.transform.translation.z = 0.0;
    static_transform_morty.transform.rotation.x = 0.0;
    static_transform_morty.transform.rotation.y = 0.0;
    static_transform_morty.transform.rotation.z = 0.0;
    static_transform_morty.transform.rotation.w = 1.0;

    geometry_msgs::msg::TransformStamped static_transform_rick;
    static_transform_rick.header.stamp = node->now();
    static_transform_rick.header.frame_id = "world";
    static_transform_rick.child_frame_id = "rick/odom";
    static_transform_rick.transform.translation.x = 0.0;
    static_transform_rick.transform.translation.y = 0.0;
    static_transform_rick.transform.translation.z = 0.0;
    static_transform_rick.transform.rotation.x = 0.0;
    static_transform_rick.transform.rotation.y = 0.0;
    static_transform_rick.transform.rotation.z = 0.0;
    static_transform_rick.transform.rotation.w = 1.0;

    broadcaster.sendTransform(static_transform_morty);
    broadcaster.sendTransform(static_transform_rick);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
