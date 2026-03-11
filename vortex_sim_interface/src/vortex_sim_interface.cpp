#include "vortex_sim_interface/vortex_sim_interface.hpp"
#include <spdlog/spdlog.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

VortexSimInterface::VortexSimInterface(const rclcpp::NodeOptions& options)
    : Node("vortex_sim_interface", options) {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos_sensor_data = rclcpp::QoS(
        rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    auto drone_thruster_topic =
        this->declare_parameter<std::string>("topics.thruster_forces");
    drone_thruster_sub_ =
        this->create_subscription<vortex_msgs::msg::ThrusterForces>(
            drone_thruster_topic, qos_sensor_data,
            std::bind(&VortexSimInterface::thruster_callback, this,
                      std::placeholders::_1));

    stonefish_thruster_pub_ =
        this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "stonefish/thrusters", 10);

    std::string odom_topic =
        this->declare_parameter<std::string>("topics.odom");
    std::string pose_topic =
        this->declare_parameter<std::string>("topics.pose");
    std::string twist_topic =
        this->declare_parameter<std::string>("topics.twist");

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, qos_sensor_data,
        std::bind(&VortexSimInterface::odom_callback, this,
                  std::placeholders::_1));

    pose_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            pose_topic, qos_sensor_data);
    twist_pub_ =
        this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
            twist_topic, qos_sensor_data);

    dvl_sub_ = this->create_subscription<stonefish_ros2::msg::DVL>(
        "dvl/sim", qos_sensor_data,
        std::bind(&VortexSimInterface::dvl_callback, this,
                  std::placeholders::_1));
    std::string dvl_twist_topic =
        this->declare_parameter<std::string>("topics.dvl_twist");
    dvl_twist_pub_ =
        this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
            dvl_twist_topic, qos_sensor_data);

    std::string stonefish_sonar_info_topic = "fls_image/stonefish_sonar_info";

    std::string vortex_sonar_info_topic =
        this->declare_parameter<std::string>("topics.sonar_info", "");

    if (vortex_sonar_info_topic.empty()) {
        spdlog::warn(
            "Parameter 'topics.sonar_info' is empty. "
            "Sonar info will not be published as vortex msg.");
    } else {
        sonar_info_stonefish_sub_ =
            this->create_subscription<stonefish_ros2::msg::SonarInfo>(
                stonefish_sonar_info_topic, qos_sensor_data,
                std::bind(&VortexSimInterface::sonar_info_callback, this,
                          std::placeholders::_1));

        sonar_info_vortex_pub_ =
            this->create_publisher<vortex_msgs::msg::SonarInfo>(
                vortex_sonar_info_topic, qos_sensor_data);
    }

    std::string dvl_altitude_topic =
        this->declare_parameter<std::string>("topics.dvl_altitude");
    dvl_altitude_pub_ = this->create_publisher<vortex_msgs::msg::DVLAltitude>(
        dvl_altitude_topic, qos_sensor_data);

    mock_odom_ = this->declare_parameter<bool>("mock_odom");
    tf_name_prefix_ = this->declare_parameter<std::string>("tf_name_prefix");
    if (mock_odom_) {
        odom_tf_pub_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        spdlog::info(
            "Mock odometry is enabled. Odometry will be published and TF "
            "frames will be published with prefix '{}'",
            tf_name_prefix_);
    }
}

void VortexSimInterface::thruster_callback(
    const vortex_msgs::msg::ThrusterForces::SharedPtr msg) {
    if (msg->thrust.size() != 8) {
        spdlog::error(
            "Received ThrusterForces with incorrect size. "
            "Expected 8, got {}",
            msg->thrust.size());
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

void VortexSimInterface::odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
    geometry_msgs::msg::PoseWithCovarianceStamped posestamped_msg;
    posestamped_msg.header = odom_msg->header;
    posestamped_msg.pose = odom_msg->pose;

    geometry_msgs::msg::TwistWithCovarianceStamped twist_msg;
    twist_msg.header = odom_msg->header;
    twist_msg.twist = odom_msg->twist;

    pose_pub_->publish(posestamped_msg);
    twist_pub_->publish(twist_msg);

    if (mock_odom_) {
        geometry_msgs::msg::TransformStamped transform_stamped;

        transform_stamped.header.stamp = odom_msg->header.stamp;
        transform_stamped.header.frame_id = tf_name_prefix_ + "/odom";
        transform_stamped.child_frame_id = tf_name_prefix_ + "/base_link";

        transform_stamped.transform.translation.x =
            odom_msg->pose.pose.position.x;
        transform_stamped.transform.translation.y =
            odom_msg->pose.pose.position.y;
        transform_stamped.transform.translation.z =
            odom_msg->pose.pose.position.z;

        transform_stamped.transform.rotation = odom_msg->pose.pose.orientation;

        tf2::Quaternion q;
        q.setX(odom_msg->pose.pose.orientation.x);
        q.setY(odom_msg->pose.pose.orientation.y);
        q.setZ(odom_msg->pose.pose.orientation.z);
        q.setW(odom_msg->pose.pose.orientation.w);

        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        odom_tf_pub_->sendTransform(transform_stamped);
    }
}

void VortexSimInterface::dvl_callback(
    const stonefish_ros2::msg::DVL::SharedPtr dvl_msg) {
    geometry_msgs::msg::TwistWithCovarianceStamped twist_msg;
    twist_msg.header = dvl_msg->header;
    twist_msg.twist.twist.linear.x = dvl_msg->velocity.x;
    twist_msg.twist.twist.linear.y = dvl_msg->velocity.y;
    twist_msg.twist.twist.linear.z = dvl_msg->velocity.z;
    twist_msg.twist.covariance[0] = dvl_msg->velocity_covariance[0];
    twist_msg.twist.covariance[7] = dvl_msg->velocity_covariance[4];
    twist_msg.twist.covariance[14] = dvl_msg->velocity_covariance[8];
    dvl_twist_pub_->publish(twist_msg);

    vortex_msgs::msg::DVLAltitude altitude_msg;
    altitude_msg.header = dvl_msg->header;
    altitude_msg.altitude = dvl_msg->altitude;
    dvl_altitude_pub_->publish(altitude_msg);
}

void VortexSimInterface::sonar_info_callback(
    const stonefish_ros2::msg::SonarInfo::SharedPtr sonar_msg) {
    vortex_msgs::msg::SonarInfo vortex_sonar_msg;

    vortex_sonar_msg.header = sonar_msg->header;
    vortex_sonar_msg.height = sonar_msg->height;
    vortex_sonar_msg.width = sonar_msg->width;
    vortex_sonar_msg.meters_per_pixel_x = sonar_msg->meters_per_pixel_x;
    vortex_sonar_msg.meters_per_pixel_y = sonar_msg->meters_per_pixel_y;
    vortex_sonar_msg.max_range = sonar_msg->max_range;
    vortex_sonar_msg.min_range = sonar_msg->min_range;
    vortex_sonar_msg.vertical_fov = sonar_msg->vertical_fov;

    sonar_info_vortex_pub_->publish(vortex_sonar_msg);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VortexSimInterface>());
    rclcpp::shutdown();
    return 0;
}
