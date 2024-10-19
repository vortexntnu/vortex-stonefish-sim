#include <rclcpp/rclcpp.hpp>
#include <vortex_msgs/msg/thruster_forces.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <Eigen/Geometry>
#include <vector>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>


class VortexSimInterface : public rclcpp::Node
{
public:
    VortexSimInterface() : Node("vortex_sim_interface")
    {
        subscription_ = this->create_subscription<vortex_msgs::msg::ThrusterForces>(
            "thrust/thruster_forces", 10, std::bind(&VortexSimInterface::thruster_callback, this, std::placeholders::_1));

        stonefish_thruster_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/stonefish/thrusters", 10);

        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "nucleus/odom", 10, std::bind(&VortexSimInterface::odom_callback, this, std::placeholders::_1));

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        publish_camera_down_transform();

        RCLCPP_INFO(this->get_logger(), "VortexSimInterface has been started.");
    }

private:
    rclcpp::Subscription<vortex_msgs::msg::ThrusterForces>::SharedPtr subscription_;

    // Add Transform Broadcaster and Timer
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr stonefish_thruster_pub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

    void thruster_callback(const vortex_msgs::msg::ThrusterForces::SharedPtr msg)
    {
        if (msg->thrust.size() != 8) {
            RCLCPP_ERROR(this->get_logger(), "Received ThrusterForces with incorrect size. Expected 8, got %zu", msg->thrust.size());
            return;
        }
        
        std_msgs::msg::Float64MultiArray thrust_array_msg;

        thrust_array_msg.layout.dim.resize(1);
        thrust_array_msg.layout.dim[0].size = 8;
        thrust_array_msg.layout.dim[0].stride = 1;

        thrust_array_msg.data.resize(8); 

        for (size_t i = 0; i < 8; ++i) {
            thrust_array_msg.data[i] = msg->thrust[i];
        }

        stonefish_thruster_pub_->publish(thrust_array_msg);
    }

    
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg_nuw)
    {
        geometry_msgs::msg::TransformStamped transform_stamped;

        transform_stamped.header.stamp = odom_msg_nuw->header.stamp;
        transform_stamped.header.frame_id = "odom";
        transform_stamped.child_frame_id = "base_link";

        transform_stamped.transform.translation.x = odom_msg_nuw->pose.pose.position.x;
        transform_stamped.transform.translation.y = odom_msg_nuw->pose.pose.position.y;
        transform_stamped.transform.translation.z = odom_msg_nuw->pose.pose.position.z;

        transform_stamped.transform.rotation = odom_msg_nuw->pose.pose.orientation;

        tf_broadcaster_->sendTransform(transform_stamped);
    }

     void publish_camera_down_transform()
    {
        geometry_msgs::msg::TransformStamped transform_stamped;

        transform_stamped.header.stamp = this->get_clock()->now();
        transform_stamped.header.frame_id = "base_link";
        transform_stamped.child_frame_id = "Orca/camera_down";

        transform_stamped.transform.translation.x = 0.4;
        transform_stamped.transform.translation.y = 0.0;
        transform_stamped.transform.translation.z = 0.2;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, 1.571);
        transform_stamped.transform.rotation.x = q.x();
        transform_stamped.transform.rotation.y = q.y();
        transform_stamped.transform.rotation.z = q.z();
        transform_stamped.transform.rotation.w = q.w();

        static_tf_broadcaster_->sendTransform(transform_stamped);

    }

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VortexSimInterface>());
    rclcpp::shutdown();
    return 0;
}