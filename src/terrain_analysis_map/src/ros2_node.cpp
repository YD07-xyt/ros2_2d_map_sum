#include"../include/ros2_node.hpp"
#include <memory>
#include <rclcpp/utilities.hpp>

int main(int argc, const char *const *argv){
    rclcpp::init(argc,argv);
    auto nh = std::make_shared<rclcpp::Node>("terrain_analysis_map_node");
    TerrainAnalysis::Ros2Node ros2_node_class(nh);
    rclcpp::spin(nh);
    rclcpp::shutdown();
    return 0;
}