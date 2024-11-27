#include "tf2/transform_datatypes.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp" //added
#include "rclcpp/node_options.hpp" //added
#include <tf2/LinearMath/Transform.h>
#include "whycon/localization_system.hpp" //recheck
//#include <boost/shared_ptr.hpp> 
/*ROS 2 provides its own smart pointer types for managing resources and objects.
The primary smart pointer used in ROS 2 is std::shared_ptr, which is part of the C++ Standard Library.*/
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "image_geometry/pinhole_camera_model.h"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/pose_array.hpp"


namespace whycon {
  class WhyConROS : public rclcpp::Node
  {
    public:
      WhyConROS();
      //WhyConROS(const rclcpp::NodeOptions& options);//added

      void on_image(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg, const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg);

      void reset(const std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response);


    protected:
      rclcpp::Node::SharedPtr node_handle;
      image_transport::ImageTransport it;
      image_transport::CameraSubscriber cam_sub;
     


    private:
			void load_transforms(void);
      void publish_results(const std_msgs::msg::Header& header, const cv_bridge::CvImageConstPtr& cv_ptr);  //review later
      
			whycon::DetectorParameters parameters;
      std::shared_ptr<whycon::LocalizationSystem> system;
      bool is_tracking, should_reset;
      int max_attempts, max_refine;
      std::string world_frame_id, frame_id;
			int targets;
      double xscale, yscale;
      double inner_diameter, outer_diameter;

	
      rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service;  //review later

      image_transport::Publisher image_pub;
      rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr poses_pub;
      image_transport::Publisher context_pub;
    

      image_geometry::PinholeCameraModel camera_model;

  };
}
