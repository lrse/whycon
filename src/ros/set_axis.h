#ifndef SET_AXIS_NODE_H
#define SET_AXIS_NODE_H

#include <ros/ros.h>
#include <whycon/localization_system.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_subscriber.h>
#include <std_srvs/Empty.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>

namespace whycon {
  class AxisSetter {
    public:
      AxisSetter(ros::NodeHandle& n);

      boost::shared_ptr<cv::LocalizationSystem> system;

      void on_image(const sensor_msgs::ImageConstPtr& img_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
      bool set_axis(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

      bool set_axis_now, is_tracking;
      std::string axis_name;
      double outer_diameter, inner_diameter;

      image_transport::ImageTransport it;
      image_transport::CameraSubscriber cam_sub;
      ros::Publisher img_pub;
      ros::ServiceServer set_axis_service;

      image_geometry::PinholeCameraModel camera_model;
  };
}

#endif // SET_AXIS_NODE_H
