#ifndef STONEFISH_SIM_INTERFACE__STONEFISH_SIM_INTERFACE_HPP_
#define STONEFISH_SIM_INTERFACE__STONEFISH_SIM_INTERFACE_HPP_

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <stonefish_ros2/msg/dvl.hpp>
#include <stonefish_ros2/msg/sonar_info.hpp>
#include <string>
#include <vortex_msgs/msg/dvl_altitude.hpp>
#include <vortex_msgs/msg/sonar_info.hpp>
#include <vortex_msgs/msg/thruster_forces.hpp>

class StonefishSimInterface : public rclcpp::Node {
   public:
    explicit StonefishSimInterface(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

   private:
    void thruster_callback(
        const vortex_msgs::msg::ThrusterForces::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
    void dvl_callback(const stonefish_ros2::msg::DVL::SharedPtr dvl_msg);
    void sonar_info_callback(
        const stonefish_ros2::msg::SonarInfo::SharedPtr sonar_msg);

    rclcpp::Subscription<vortex_msgs::msg::ThrusterForces>::SharedPtr
        drone_thruster_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
        stonefish_thruster_pub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
        pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
        twist_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> odom_tf_pub_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> odom_enu_tf_pub_;

    rclcpp::Subscription<stonefish_ros2::msg::DVL>::SharedPtr dvl_sub_;
    rclcpp::Publisher<vortex_msgs::msg::DVLAltitude>::SharedPtr
        dvl_altitude_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
        dvl_twist_pub_;

    rclcpp::Subscription<stonefish_ros2::msg::SonarInfo>::SharedPtr
        sonar_info_stonefish_sub_;
    rclcpp::Publisher<vortex_msgs::msg::SonarInfo>::SharedPtr
        sonar_info_vortex_pub_;

    bool mock_odom_{false};
    std::string tf_name_prefix_;
};

#endif  // STONEFISH_SIM_INTERFACE__STONEFISH_SIM_INTERFACE_HPP_
