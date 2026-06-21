#pragma once
#include "config.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pcl/filters/voxel_grid.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float32.hpp"
#include "terrain_analysis_map.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <memory>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <pcl_ros/transforms.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <vector>
namespace TerrainAnalysis {
class Ros2Node {
public:
  Ros2Node(rclcpp::Node::SharedPtr nh) {
    nh_ = nh;
    init_params(nh_);

    terrain_analysis_map_ = std::make_shared<TerrainAnalysisMap>(config_);
    odom_update.reserve(6);
    laserCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

    odom_sub_ = nh_->create_subscription<nav_msgs::msg::Odometry>(
        config_.odom_name, 5, [this](nav_msgs::msg::Odometry::SharedPtr data) {
          odometryCallback(data);
        });

    laser_sub_ = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
        config_.cloud_name, 5,
        [this](sensor_msgs::msg::PointCloud2::SharedPtr data) {
          laserCloudCallback(data);
        });

    laser_pub_ =
        nh_->create_publisher<sensor_msgs::msg::PointCloud2>("terrain_map", 2);

    timer_ = nh_->create_wall_timer(std::chrono::milliseconds(100),
                                    [this]() { TerrainAnalysisCallback(); });
  };

private:
  std::shared_ptr<TerrainAnalysisMap> terrain_analysis_map_;

  rclcpp::Node::SharedPtr nh_;
  TerrainAnalysisConfig config_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr laser_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr laser_pub_;
  std::vector<double> odom_update;

  double systemInitTime = 0;
  bool systemInited = false;
  int noDataInited = 0;

  double laserCloudTime = 0;
  bool newlaserCloud = false;
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud;

private:
  void TerrainAnalysisCallback() {
    if (newlaserCloud) {
      newlaserCloud = false;
      terrain_analysis_map_->updateMap();
      auto pc = terrain_analysis_map_->get_pointcloud();
      // publish points with elevation
      sensor_msgs::msg::PointCloud2 terrainCloud2;
      pcl::toROSMsg(*pc, terrainCloud2);
      terrainCloud2.header.stamp =
          rclcpp::Time(static_cast<uint64_t>(laserCloudTime * 1e9));
      terrainCloud2.header.frame_id = "world";
      laser_pub_->publish(terrainCloud2);
    }
  };
  // state estimation callback function
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr odom) {
    odom_update.clear();
    double roll, pitch, yaw;
    geometry_msgs::msg::Quaternion geoQuat = odom->pose.pose.orientation;
    tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w))
        .getRPY(roll, pitch, yaw);

    odom_update.push_back(odom->pose.pose.position.x);
    odom_update.push_back(odom->pose.pose.position.y);
    odom_update.push_back(odom->pose.pose.position.z);
    odom_update.push_back(roll);
    odom_update.push_back(pitch);
    odom_update.push_back(yaw);
    terrain_analysis_map_->updateOdom(odom_update);
  }

  // registered laser scan callback function
  void laserCloudCallback(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloud2) {

    laserCloudTime = rclcpp::Time(laserCloud2->header.stamp).seconds();
    if (!systemInited) {
      systemInitTime = laserCloudTime;
      systemInited = true;
    }
    terrain_analysis_map_->updateTime(laserCloudTime, systemInitTime);
    laserCloud->clear();
    pcl::fromROSMsg(*laserCloud2, *laserCloud);

    terrain_analysis_map_->updatelaserCloud(laserCloud);

    newlaserCloud = true;
  }
  void init_params(rclcpp::Node::SharedPtr nh) {

    nh->declare_parameter<double>("scanVoxelSize", config_.scanVoxelSize);
    nh->declare_parameter<double>("decayTime", config_.decayTime);
    nh->declare_parameter<double>("noDecayDis", config_.noDecayDis);
    nh->declare_parameter<double>("clearingDis", config_.clearingDis);
    nh->declare_parameter<bool>("useSorting", config_.useSorting);
    nh->declare_parameter<double>("quantileZ", config_.quantileZ);
    nh->declare_parameter<bool>("considerDrop", config_.considerDrop);
    nh->declare_parameter<bool>("limitGroundLift", config_.limitGroundLift);
    nh->declare_parameter<double>("maxGroundLift", config_.maxGroundLift);
    nh->declare_parameter<bool>("clearDyObs", config_.clearDyObs);
    nh->declare_parameter<double>("minDyObsDis", config_.minDyObsDis);
    nh->declare_parameter<double>("minDyObsAngle", config_.minDyObsAngle);
    nh->declare_parameter<double>("minDyObsRelZ", config_.minDyObsRelZ);
    nh->declare_parameter<double>("absDyObsRelZThre", config_.absDyObsRelZThre);
    nh->declare_parameter<double>("minDyObsVFOV", config_.minDyObsVFOV);
    nh->declare_parameter<double>("maxDyObsVFOV", config_.maxDyObsVFOV);
    nh->declare_parameter<int>("minDyObsPointNum", config_.minDyObsPointNum);
    nh->declare_parameter<bool>("noDataObstacle", config_.noDataObstacle);
    nh->declare_parameter<int>("noDataBlockSkipNum",
                               config_.noDataBlockSkipNum);
    nh->declare_parameter<int>("minBlockPointNum", config_.minBlockPointNum);
    nh->declare_parameter<double>("vehicleHeight", config_.vehicleHeight);
    nh->declare_parameter<int>("voxelPointUpdateThre",
                               config_.voxelPointUpdateThre);
    nh->declare_parameter<double>("voxelTimeUpdateThre",
                                  config_.voxelTimeUpdateThre);
    nh->declare_parameter<double>("minRelZ", config_.minRelZ);
    nh->declare_parameter<double>("maxRelZ", config_.maxRelZ);
    nh->declare_parameter<double>("disRatioZ", config_.disRatioZ);
    nh->declare_parameter<std::string>("cloud_topic_name", config_.cloud_name);
    // odometry_name
    nh->declare_parameter<std::string>("odom_topic_name", config_.odom_name);
    nh->get_parameter("scanVoxelSize", config_.scanVoxelSize);
    nh->get_parameter("decayTime", config_.decayTime);
    nh->get_parameter("noDecayDis", config_.noDecayDis);
    nh->get_parameter("clearingDis", config_.clearingDis);
    nh->get_parameter("useSorting", config_.useSorting);
    nh->get_parameter("quantileZ", config_.quantileZ);
    nh->get_parameter("considerDrop", config_.considerDrop);
    nh->get_parameter("limitGroundLift", config_.limitGroundLift);
    nh->get_parameter("maxGroundLift", config_.maxGroundLift);
    nh->get_parameter("clearDyObs", config_.clearDyObs);
    nh->get_parameter("minDyObsDis", config_.minDyObsDis);
    nh->get_parameter("minDyObsAngle", config_.minDyObsAngle);
    nh->get_parameter("minDyObsRelZ", config_.minDyObsRelZ);
    nh->get_parameter("absDyObsRelZThre", config_.absDyObsRelZThre);
    nh->get_parameter("minDyObsVFOV", config_.minDyObsVFOV);
    nh->get_parameter("maxDyObsVFOV", config_.maxDyObsVFOV);
    nh->get_parameter("minDyObsPointNum", config_.minDyObsPointNum);
    nh->get_parameter("noDataObstacle", config_.noDataObstacle);
    nh->get_parameter("noDataBlockSkipNum", config_.noDataBlockSkipNum);
    nh->get_parameter("minBlockPointNum", config_.minBlockPointNum);
    nh->get_parameter("vehicleHeight", config_.vehicleHeight);
    nh->get_parameter("voxelPointUpdateThre", config_.voxelPointUpdateThre);
    nh->get_parameter("voxelTimeUpdateThre", config_.voxelTimeUpdateThre);
    nh->get_parameter("minRelZ", config_.minRelZ);
    nh->get_parameter("maxRelZ", config_.maxRelZ);
    nh->get_parameter("disRatioZ", config_.disRatioZ);
    nh->get_parameter("cloud_topic_name", config_.cloud_name);
    nh->get_parameter("odom_topic_name", config_.odom_name);
  };
};
} // namespace TerrainAnalysis