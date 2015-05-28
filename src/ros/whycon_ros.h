#include <ros/ros.h>
#include <whycon/localization_system.h>
#include <boost/shared_ptr.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <std_srvs/Empty.h>

namespace whycon {
  class WhyConROS {
    public:
      WhyConROS(ros::NodeHandle& n);

      void on_image(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
      bool reset(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

    private:
      void publish_results(const std_msgs::Header& header, const cv_bridge::CvImageConstPtr& cv_ptr);

      int targets;
      double outer_diameter, inner_diameter;
      double xscale, yscale;
      
      boost::shared_ptr<cv::LocalizationSystem> system;
      bool is_tracking, should_reset;
      int max_attempts, max_refine;
      std::string axis_file;

      image_transport::ImageTransport it;
      image_transport::CameraSubscriber cam_sub;
      ros::ServiceServer reset_service;

      ros::Publisher image_pub, viz_pub, poses_pub, trans_poses_pub, points_pub, context_pub;

      image_geometry::PinholeCameraModel camera_model;
  };
}
