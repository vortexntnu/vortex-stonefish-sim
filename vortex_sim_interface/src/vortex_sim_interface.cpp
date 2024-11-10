#include <rclcpp/rclcpp.hpp>
#include <vortex_msgs/msg/thruster_forces.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <Eigen/Geometry>
#include <vector>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <stonefish_ros2/msg/dvl.hpp>


class VortexSimInterface : public rclcpp::Node
{
public:
    VortexSimInterface() : Node("vortex_sim_interface")
    {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos_sensor_data = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);
        subscription_ = this->create_subscription<vortex_msgs::msg::ThrusterForces>(
            "thrust/thruster_forces", qos_sensor_data, std::bind(&VortexSimInterface::thruster_callback, this, std::placeholders::_1));

        stonefish_thruster_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/stonefish/thrusters", qos_sensor_data);

        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "nucleus/odom", qos_sensor_data, std::bind(&VortexSimInterface::odom_callback, this, std::placeholders::_1));

        odom_euler_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/nucleus/odom_euler", qos_sensor_data);

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/nucleus/pose", qos_sensor_data);

        dvl_subscriber_ = this->create_subscription<stonefish_ros2::msg::DVL>(
            "dvl/sim", qos_sensor_data, std::bind(&VortexSimInterface::dvl_callback, this, std::placeholders::_1));

        depth_pub_ = this->create_publisher<std_msgs::msg::Float64>("/dvl/depth", qos_sensor_data);

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        depth_cam_tf_pub_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        odom_tf_pub_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        odom_height_tf_pub_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        orca_cam_front_tf_pub_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        sonar_tf_pub_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        world_enu_tf_pub_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        pubish_depth_cam_transform();
        publish_camera_down_transform();
        publish_world_odom_tf();
        pubish_sonar_transform();

        

        RCLCPP_INFO(this->get_logger(), "VortexSimInterface has been started.");
    }

private:
    rclcpp::Subscription<vortex_msgs::msg::ThrusterForces>::SharedPtr subscription_;

    // Add Transform Broadcaster and Timer
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> depth_cam_tf_pub_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> odom_height_tf_pub_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> odom_tf_pub_;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> orca_cam_front_tf_pub_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> sonar_tf_pub_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> world_enu_tf_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr stonefish_thruster_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr odom_euler_pub_;
    
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr depth_pub_;
    rclcpp::Subscription<stonefish_ros2::msg::DVL>::SharedPtr dvl_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

    void dvl_callback(const stonefish_ros2::msg::DVL::SharedPtr dvl_msg)
    {
        std_msgs::msg::Float64 depth_msg;
        depth_msg.data = dvl_msg->altitude;
        depth_pub_->publish(depth_msg);
    }

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

    void publish_odom_height_tf(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
    {
        geometry_msgs::msg::TransformStamped transform_stamped;

        transform_stamped.header.stamp = odom_msg->header.stamp;
        transform_stamped.header.frame_id = "world_ned";
        transform_stamped.child_frame_id = "odom_height";

        transform_stamped.transform.translation.x = 0.0;
        transform_stamped.transform.translation.y = 0.0;
        transform_stamped.transform.translation.z = odom_msg->pose.pose.position.z + 2.0;

        odom_height_tf_pub_->sendTransform(transform_stamped);
    }

    
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
    {
        geometry_msgs::msg::TransformStamped transform_stamped;

        transform_stamped.header.stamp = odom_msg->header.stamp;
        transform_stamped.header.frame_id = "odom";
        transform_stamped.child_frame_id = "base_link";

        transform_stamped.transform.translation.x = odom_msg->pose.pose.position.x;
        transform_stamped.transform.translation.y = odom_msg->pose.pose.position.y;
        transform_stamped.transform.translation.z = odom_msg->pose.pose.position.z;

        transform_stamped.transform.rotation = odom_msg->pose.pose.orientation;

        // Convert quaternions to euler
        tf2::Quaternion q;

        q.setX(odom_msg->pose.pose.orientation.x);
        q.setY(odom_msg->pose.pose.orientation.y);
        q.setZ(odom_msg->pose.pose.orientation.z);
        q.setW(odom_msg->pose.pose.orientation.w);

        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        std_msgs::msg::Float64MultiArray euler_msg;
        euler_msg.data.push_back(roll);
        euler_msg.data.push_back(pitch);
        euler_msg.data.push_back(yaw);

        odom_euler_pub_->publish(euler_msg);

        tf_broadcaster_->sendTransform(transform_stamped);

        geometry_msgs::msg::TransformStamped transform_stamped1;

        transform_stamped1.header.stamp = odom_msg->header.stamp;
        transform_stamped1.header.frame_id = "odom";
        transform_stamped1.child_frame_id = "nvblox_height";

        transform_stamped1.transform.translation.x = 0.0;
        transform_stamped1.transform.translation.y = 0.0;
        transform_stamped1.transform.translation.z = odom_msg->pose.pose.position.z;

        tf2::Quaternion q1;
        q1.setRPY(0.0, 0.0, 0.0);
        transform_stamped1.transform.rotation.x = q1.x();
        transform_stamped1.transform.rotation.y = q1.y();
        transform_stamped1.transform.rotation.z = q1.z();
        transform_stamped1.transform.rotation.w = q1.w();


        odom_height_tf_pub_->sendTransform(transform_stamped1);

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header = odom_msg->header;
        pose_msg.pose.position = odom_msg->pose.pose.position;
        pose_msg.pose.orientation = odom_msg->pose.pose.orientation;

        pose_pub_->publish(pose_msg);
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

    void pubish_depth_cam_transform()
    {
        geometry_msgs::msg::TransformStamped transform_stamped;

        transform_stamped.header.stamp = this->get_clock()->now();
        transform_stamped.header.frame_id = "base_link";
        transform_stamped.child_frame_id = "Orca/Dcam";

        transform_stamped.transform.translation.x = 0.45;
        transform_stamped.transform.translation.y = 0.0;
        transform_stamped.transform.translation.z = -0.1;

        tf2::Quaternion q;
        q.setRPY(1.57, 0.0, 1.571);
        transform_stamped.transform.rotation.x = q.x();
        transform_stamped.transform.rotation.y = q.y();
        transform_stamped.transform.rotation.z = q.z();
        transform_stamped.transform.rotation.w = q.w();

        depth_cam_tf_pub_->sendTransform(transform_stamped);
        transform_stamped.child_frame_id = "Orca/camera_front";
        orca_cam_front_tf_pub_->sendTransform(transform_stamped);
    }

    void pubish_sonar_transform()
    {
        geometry_msgs::msg::TransformStamped transform_stamped;

        transform_stamped.header.stamp = this->get_clock()->now();
        transform_stamped.header.frame_id = "base_link";
        transform_stamped.child_frame_id = "Orca/Multibeam2";

        transform_stamped.transform.translation.x = 0.45;
        transform_stamped.transform.translation.y = 0.0;
        transform_stamped.transform.translation.z = -0.1;

        tf2::Quaternion q;
        q.setRPY(1.57, 0.0, 1.571);
        transform_stamped.transform.rotation.x = q.x();
        transform_stamped.transform.rotation.y = q.y();
        transform_stamped.transform.rotation.z = q.z();
        transform_stamped.transform.rotation.w = q.w();

        sonar_tf_pub_->sendTransform(transform_stamped);
    }

    void publish_world_odom_tf()
{
    geometry_msgs::msg::TransformStamped transform_stamped;

    transform_stamped.header.stamp = this->get_clock()->now();
    transform_stamped.header.frame_id = "world_ned";
    transform_stamped.child_frame_id = "odom";

    transform_stamped.transform.translation.x = 0.0;
    transform_stamped.transform.translation.y = 0.0;
    transform_stamped.transform.translation.z = 2.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    transform_stamped.transform.rotation.x = q.x();
    transform_stamped.transform.rotation.y = q.y();
    transform_stamped.transform.rotation.z = q.z();
    transform_stamped.transform.rotation.w = q.w();

    odom_tf_pub_->sendTransform(transform_stamped);

    geometry_msgs::msg::TransformStamped odom_ned_to_enu;
    odom_ned_to_enu.header.stamp = this->get_clock()->now();
    odom_ned_to_enu.header.frame_id = "odom";
    odom_ned_to_enu.child_frame_id = "odom_enu";
    
    odom_ned_to_enu.transform.translation.x = 0.0;
    odom_ned_to_enu.transform.translation.y = 0.0;
    odom_ned_to_enu.transform.translation.z = 0.0;

    // Create the NED to ENU transformation (90 deg rotation about Z, then 180 deg rotation about X)
    tf2::Quaternion q1;
    q1.setRPY(M_PI, 0.0, M_PI_2); // First rotate by 180 degrees about X, then -90 degrees about Z
    
    odom_ned_to_enu.transform.rotation.x = q1.x();
    odom_ned_to_enu.transform.rotation.y = q1.y();
    odom_ned_to_enu.transform.rotation.z = q1.z();
    odom_ned_to_enu.transform.rotation.w = q1.w();

    world_enu_tf_pub_->sendTransform(odom_ned_to_enu);
}


};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VortexSimInterface>());
    rclcpp::shutdown();
    return 0;
}