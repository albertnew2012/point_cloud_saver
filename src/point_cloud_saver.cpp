#include <filesystem>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <filesystem> 

class PointCloudSaver : public rclcpp::Node
{
  public:
    PointCloudSaver(const std::string &topic_name, const std::string &output_dir)
        : Node("point_cloud_saver"), output_dir_(output_dir)
    {
        // Create the output directory if it does not exist
        std::filesystem::create_directory(output_dir_);

        // Subscribe to the point cloud topic
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic_name, 10,
            std::bind(&PointCloudSaver::pointCloudCallback, this, std::placeholders::_1));
    }

  private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert the PointCloud2 message to PCL PointCloud
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*msg, pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
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

    // Parse parameters
    auto node = std::make_shared<rclcpp::Node>("point_cloud_saver");
    std::string topic_name =
        node->declare_parameter<std::string>("topic_name", "/point_cloud_topic");
    std::string output_dir = node->declare_parameter<std::string>("output_dir", "output_directory");

    auto saver = std::make_shared<PointCloudSaver>(topic_name, output_dir);
    rclcpp::spin(saver);
    rclcpp::shutdown();
    return 0;
}
