#include "stonefish_sim_interface/stonefish_sim_interface.hpp"
#include <spdlog/spdlog.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

StonefishSimInterface::StonefishSimInterface(const rclcpp::NodeOptions& options)
    : Node("stonefish_sim_interface", options) {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos_sensor_data = rclcpp::QoS(
        rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    auto drone_thruster_topic =
        this->declare_parameter<std::string>("topics.thruster_forces");
    drone_thruster_sub_ =
        this->create_subscription<vortex_msgs::msg::ThrusterForces>(
            drone_thruster_topic, qos_sensor_data,
            std::bind(&StonefishSimInterface::thruster_callback, this,
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
        std::bind(&StonefishSimInterface::odom_callback, this,
                  std::placeholders::_1));

    pose_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            pose_topic, qos_sensor_data);
    twist_pub_ =
        this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
            twist_topic, qos_sensor_data);

    dvl_sub_ = this->create_subscription<stonefish_ros2::msg::DVL>(
        "dvl/sim", qos_sensor_data,
        std::bind(&StonefishSimInterface::dvl_callback, this,
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
                std::bind(&StonefishSimInterface::sonar_info_callback, this,
                          std::placeholders::_1));

        sonar_info_vortex_pub_ =
            this->create_publisher<vortex_msgs::msg::SonarInfo>(
                vortex_sonar_info_topic, qos_sensor_data);
    }

    std::string dvl_altitude_topic =
        this->declare_parameter<std::string>("topics.dvl_altitude");
    dvl_altitude_pub_ = this->create_publisher<vortex_msgs::msg::DVLAltitude>(
        dvl_altitude_topic, qos_sensor_data);

    auto gripper_servos_topic =
        this->declare_parameter<std::string>("topics.gripper_servos", "");

    if (gripper_servos_topic.empty()) {
        spdlog::warn(
            "Parameter 'topics.gripper_servos' is empty. "
            "Gripper servo forwarding disabled.");
    } else {
        gripper_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            gripper_servos_topic, qos_sensor_data,
            std::bind(&StonefishSimInterface::gripper_callback, this,
                      std::placeholders::_1));

        stonefish_servo_pub_ =
            this->create_publisher<sensor_msgs::msg::JointState>(
                "stonefish/servos", rclcpp::QoS(10).reliable());
    }

    drone_name_ = this->declare_parameter<std::string>("drone", "orca");
    mock_odom_ = this->declare_parameter<bool>("mock_odom");
    tf_name_prefix_ = this->declare_parameter<std::string>("tf_name_prefix");
    if (mock_odom_) {
        odom_tf_pub_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        spdlog::info(
            "Mock odometry is enabled. Odometry will be published and TF "
            "frames will be published with prefix '{}'",
            tf_name_prefix_);
    }
    // Publish a static transform from odom (NED) to odom_enu (ENU)
    // so Foxglove 3D panel can display a top-down view.
    // NED to ENU rotation: 180° around the axis (1,1,0)/√2
    // Quaternion: w=0, x=√2/2, y=√2/2, z=0
    odom_enu_tf_pub_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    geometry_msgs::msg::TransformStamped odom_enu_transform;
    odom_enu_transform.header.stamp = this->now();
    odom_enu_transform.header.frame_id = tf_name_prefix_ + "/odom";
    odom_enu_transform.child_frame_id = tf_name_prefix_ + "/odom_enu";
    odom_enu_transform.transform.translation.x = 0.0;
    odom_enu_transform.transform.translation.y = 0.0;
    odom_enu_transform.transform.translation.z = 0.0;
    odom_enu_transform.transform.rotation.x = M_SQRT1_2;
    odom_enu_transform.transform.rotation.y = M_SQRT1_2;
    odom_enu_transform.transform.rotation.z = 0.0;
    odom_enu_transform.transform.rotation.w = 0.0;
    odom_enu_tf_pub_->sendTransform(odom_enu_transform);
}

void StonefishSimInterface::thruster_callback(
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

void StonefishSimInterface::odom_callback(
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

void StonefishSimInterface::dvl_callback(
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

void StonefishSimInterface::sonar_info_callback(
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

void StonefishSimInterface::gripper_callback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (drone_name_ == "moby") {
        // Moby finger joints are prismatic ish (0-0.05m range).
        // Scale finger velocities so the joystick's -1/0/1 values
        // produce reasonable linear speeds.
        constexpr double prismatic_scale = 0.033;
        auto scaled_msg = *msg;
        for (size_t i = 0; i < scaled_msg.name.size(); ++i) {
            if (i < scaled_msg.velocity.size() &&
                scaled_msg.name[i].find("finger") != std::string::npos) {
                scaled_msg.velocity[i] *= prismatic_scale;
            }
        }
        stonefish_servo_pub_->publish(scaled_msg);
    } else {
        stonefish_servo_pub_->publish(*msg);
    }
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StonefishSimInterface>());
    rclcpp::shutdown();
    return 0;
}
