#include "rclcpp/rclcpp.hpp"
#include "whycon_ros.hpp"

/*Instead of passing the node’s name to the library initialization call,
 we do the initialization, then pass the node name to the creation of the node object */

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node_handle = std::make_shared<whycon::WhyConROS>();
  rclcpp::spin(node_handle);
  rclcpp::shutdown();
  return 0; 
}