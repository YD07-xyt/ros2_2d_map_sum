#include "uneven_map/uneven_map.h"

using namespace uneven_planner;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("uneven_map_node");
  //auto UnevenMap = std::make_shared<UnevenMap>();
  UnevenMap UnevenMap;
  UnevenMap.init(node); 
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}