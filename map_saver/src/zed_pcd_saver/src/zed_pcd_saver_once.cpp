# This cpp node saves point cloud data from 


#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

class ZedPcdSaverOnce : public rclcpp::Node
{
public:
  ZedPcdSaverOnce()
  : Node("zed_pcd_saver_once"),
    saved_(false)
  {
    // Declare a parameter for the output filename (default "zed_cloud.pcd")
    declare_parameter<std::string>("output_filename", "zed_cloud.pcd");
    get_parameter("output_filename", output_filename_);

    // subscribe to fused-point-cloud topic
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/zed/zed_node/mapping/fused_cloud",
      10,
      std::bind(&ZedPcdSaverOnce::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Waiting for one fused cloud on /zed/mapping/fused_cloud ...");
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // only saves once
    if (saved_) {
      return;
    }

    // Convert ROS2 PointCloud2 into PCL PointCloud
    pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    pcl::fromROSMsg(*msg, cloud);

    // Save to disk as binary PCD
    if (pcl::io::savePCDFileBinary(output_filename_, cloud) == 0) {
      RCLCPP_INFO(get_logger(),
                  "Saved fused cloud to '%s' (%zu points), shutting down...",
                  output_filename_.c_str(),
                  cloud.size());
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to save PCD file");
    }

    saved_ = true;

    rclcpp::shutdown();
  }

  std::string output_filename_;
  bool saved_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZedPcdSaverOnce>());
  // rclcpp::spin will exit once shutdown() is called in the callback
  return 0;
}
