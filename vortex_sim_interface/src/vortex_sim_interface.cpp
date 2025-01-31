#include <rclcpp/rclcpp.hpp>
#include <vortex_msgs/msg/thruster_forces.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <Eigen/Geometry>
#include <vector>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <stonefish_ros2/msg/dvl.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/image_transport.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


class VortexSimInterface : public rclcpp::Node
{
public:
    VortexSimInterface() : Node("vortex_sim_interface")
    {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos_sensor_data = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);
        subscription_ = this->create_subscription<vortex_msgs::msg::ThrusterForces>(
            "thrust/thruster_forces", qos_sensor_data, std::bind(&VortexSimInterface::thruster_callback, this, std::placeholders::_1));

        stonefish_thruster_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/stonefish/thrusters", 10);

        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/orca/odom", qos_sensor_data, std::bind(&VortexSimInterface::odom_callback, this, std::placeholders::_1));

        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/dvl/pose", qos_sensor_data);
        twist_publisher_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
            "/dvl/twist", qos_sensor_data);

        odom_euler_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/nucleus/odom_euler", qos_sensor_data);

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/nucleus/pose", qos_sensor_data);

        dvl_subscriber_ = this->create_subscription<stonefish_ros2::msg::DVL>(
            "/dvl/sim", qos_sensor_data, std::bind(&VortexSimInterface::dvl_callback, this, std::placeholders::_1));

        depth_pub_ = this->create_publisher<std_msgs::msg::Float64>("/dvl/depth", qos_sensor_data);

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        depth_cam_tf_pub_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        odom_tf_pub_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        odom_height_tf_pub_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        orca_cam_front_tf_pub_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        sonar_tf_pub_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        world_enu_tf_pub_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        depth_subscriber_ = image_transport::create_subscription(
            this, "/depth_cam/image_depth",
            std::bind(&VortexSimInterface::depthImageCallback, this, std::placeholders::_1),
            "raw");

        camera_info_subscriber_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/depth_cam/camera_info", 10,
            std::bind(&VortexSimInterface::cameraInfoCallback, this, std::placeholders::_1));

        // Publisher for point cloud
        point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/point_cloud", 10);

                // Initialize camera parameters
        fx_ = fy_ = cx_ = cy_ = 0.0;
        camera_info_received_ = false;

        laserscan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

        // LaserScan parameters
        angle_min_ = -M_PI / 2;       // -90 degrees
        angle_max_ = M_PI / 2;        // +90 degrees
        angle_increment_ = 0.01;      // Angular resolution (rad)
        range_min_ = 0.1;             // Minimum range (meters)
        range_max_ = 10.0;

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

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_publisher_;

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

        geometry_msgs::msg::PoseWithCovarianceStamped posestamped_msg;
        posestamped_msg.header = odom_msg->header;
        posestamped_msg.pose = odom_msg->pose;

        geometry_msgs::msg::TwistWithCovarianceStamped twist_msg;
        twist_msg.header = odom_msg->header;
        twist_msg.twist = odom_msg->twist;

        pose_publisher_->publish(posestamped_msg);
        twist_publisher_->publish(twist_msg);

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


    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        fx_ = msg->k[0]; // Focal length in x-axis
        fy_ = msg->k[4]; // Focal length in y-axis
        cx_ = msg->k[2]; // Principal point x-coordinate
        cy_ = msg->k[5]; // Principal point y-coordinate
        camera_info_received_ = true;
        // RCLCPP_INFO(this->get_logger(), "Camera info received: fx=%f, fy=%f, cx=%f, cy=%f", fx_, fy_, cx_, cy_);
    }


    void depthImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
        {
            if (!camera_info_received_)
            {
                RCLCPP_WARN(this->get_logger(), "Camera info not yet received. Skipping depth image processing.");
                return;
            }

            // Convert ROS Image message to OpenCV format
            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
            }
            catch (cv_bridge::Exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }

            // Create a PointCloud2 object
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
            cloud->header.frame_id = "Orca/Dcam";
            cloud->height = cv_ptr->image.rows;
            cloud->width = cv_ptr->image.cols;
            cloud->is_dense = false;
            cloud->points.resize(cloud->height * cloud->width);

            // Populate the point cloud
            for (int v = 0; v < cv_ptr->image.rows; ++v)
            {
                for (int u = 0; u < cv_ptr->image.cols; ++u)
                {
                    float depth = cv_ptr->image.at<float>(v, u);
                    if (std::isnan(depth) || depth <= 0.0)
                    {
                        continue; // Skip invalid points
                    }

                    pcl::PointXYZ &pt = cloud->points[v * cv_ptr->image.cols + u];
                    pt.x = (u - cx_) * depth / fx_;
                    pt.y = (v - cy_) * depth / fy_;
                    pt.z = depth;
                }
            }

            // Convert PCL PointCloud to ROS PointCloud2
            sensor_msgs::msg::PointCloud2 output;
            pcl::toROSMsg(*cloud, output);
            output.header.stamp = msg->header.stamp;
            output.header.frame_id = "Orca/Dcam";

            // Publish the point cloud
            point_cloud_publisher_->publish(output);

            // // ---------------------------------------------
            // // Convert the point cloud to a LaserScan message
            // // ---------------------------------------------


            // LaserScan configuration
            double angle_min_ = -M_PI / 4;  
            double angle_max_ = M_PI / 4;   
            double angle_increment_ = M_PI / 180.0; // 1-degree resolution
            double range_min_ = 0.000002;
            double range_max_ = 100000.0;
            double min_z_ = -0.05; 
            double max_z_ = 0.05;  





            // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::fromROSMsg(output, *cloud);



            // if (cloud->points.empty())
            // {
            //     RCLCPP_WARN(this->get_logger(), "Converted PointCloud2 contains no valid points!");
            //     return;
            // }
            // for (size_t i = 0; i < cloud->points.size(); ++i)
            // {
            //     const auto &point = cloud->points[i];

            //     // Print only valid points where x, y, and z are above 0
            //     if (point.x > 0.0 && point.y > 0.0 && point.z > 0.0)
            //     {
            //         RCLCPP_INFO(this->get_logger(), "Valid Point %ld: x=%.2f, y=%.2f, z=%.2f", 
            //                     i, point.x, point.y, point.z);
            //     }
            // }

            // for (size_t i = 0; i < output.data.size(); i += 16) // Each point has 16 bytes
            // {
            //     float* point = reinterpret_cast<float*>(&output.data[i]);

            //     float x = point[0];  // x coordinate
            //     float y = point[1];  // y coordinate
            //     float z = point[2];  // z coordinate

            //     // Print valid points where x, y, and z are above 0
            //     if (x > 0.0 && y > 0.0 && z > 0.0)
            //     {
            //         RCLCPP_INFO(this->get_logger(), "cloud Valid Point %ld: x=%.2f, y=%.2f, z=%.2f", 
            //                     i / 16, x, y, z);
            //     }
            // }

            // Compute the number of rays
            int num_rays = static_cast<int>((angle_max_ - angle_min_) / angle_increment_);
            std::vector<float> ranges(num_rays, std::numeric_limits<float>::infinity());

            for (const auto &point : cloud->points)
            {
                // if (point.z < min_z_ || point.z > max_z_) continue; // Filter by height

                double angle = atan2(point.y, point.x);
                // if (angle < angle_min_ || angle > angle_max_) continue;

                int index = static_cast<int>((angle - angle_min_) / angle_increment_);
                double range = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

                if (range >= range_min_ && range <= range_max_)
                {
                    ranges[index] = std::min(ranges[index], static_cast<float>(range));
                }
                // ranges[index] = std::min(ranges[index], static_cast<float>(range));
            }

            // Fill LaserScan message
            auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
            scan_msg->header = msg->header;
            scan_msg->header.frame_id = "Orca/Dcam";
            scan_msg->angle_min = angle_min_;
            scan_msg->angle_max = angle_max_;
            scan_msg->angle_increment = angle_increment_;
            scan_msg->range_min = range_min_;
            scan_msg->range_max = range_max_;
            scan_msg->ranges = ranges;


            laserscan_publisher_->publish(*scan_msg);
                 
        }


    image_transport::Subscriber depth_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;

    double fx_, fy_, cx_, cy_;
    bool camera_info_received_;

    // Publish LaserScan message
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_publisher_;

    double angle_min_;
    double angle_max_;
    double angle_increment_;
    double range_min_;
    double range_max_;


};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VortexSimInterface>());
    rclcpp::shutdown();
    return 0;
}