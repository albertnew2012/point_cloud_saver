#include <filesystem>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <stdexcept>

class PointCloudSaver : public rclcpp::Node
{
public:
    PointCloudSaver(const std::string &topic_name, const std::string &output_dir, const rclcpp::QoS &qos_settings)
        : Node("point_cloud_saver"), output_dir_(output_dir)
    {
        // Create the output directory if it does not exist
        std::filesystem::create_directories(output_dir_);

        // Subscribe to the point cloud topic
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic_name, qos_settings,
            std::bind(&PointCloudSaver::pointCloudCallback, this, std::placeholders::_1));
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert the PointCloud2 message to PCL PointCloud
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*msg, pcl_pc2);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

        // Extract the timestamp from the message
        auto timestamp = msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;

        // Generate a unique filename based on the timestamp in the message
        std::string pcd_file_path = output_dir_ + "/cloud_" + std::to_string(timestamp) + ".pcd";

        // Save the PointCloud to a PCD file
        pcl::io::savePCDFileASCII(pcd_file_path, *cloud);
        RCLCPP_INFO(this->get_logger(), "Saved %s", pcd_file_path.c_str());
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    std::string output_dir_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("point_cloud_saver");

    // Declare and get the topic_name parameter
    std::string topic_name;
    node->declare_parameter<std::string>("topic_name", "");
    node->get_parameter("topic_name", topic_name);

    if (topic_name.empty()) {
        RCLCPP_ERROR(node->get_logger(), "The 'topic_name' parameter must be provided and not empty");
        throw std::runtime_error("The 'topic_name' parameter must be provided and not empty");
    }

    // Get the current working directory
    std::string current_path = std::filesystem::current_path().string();
    std::string output_dir;
    node->declare_parameter<std::string>("output_dir", current_path);
    node->get_parameter("output_dir", output_dir);

    // QoS parameters
    int qos_history = node->declare_parameter<int>("qos_history", 10);
    std::string qos_reliability = node->declare_parameter<std::string>("qos_reliability", "best_effort");
    std::string qos_durability = node->declare_parameter<std::string>("qos_durability", "volatile");

    // Define QoS settings based on parameters
    rclcpp::QoS qos_settings(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    qos_settings.keep_last(qos_history);

    if (qos_reliability == "best_effort") {
        qos_settings.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    } else {
        qos_settings.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    }

    if (qos_durability == "volatile") {
        qos_settings.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    } else {
        qos_settings.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    }

    auto saver = std::make_shared<PointCloudSaver>(topic_name, output_dir, qos_settings);
    rclcpp::spin(saver);
    rclcpp::shutdown();
    return 0;
}