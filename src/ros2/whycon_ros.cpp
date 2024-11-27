#include "camera_info_manager/camera_info_manager.hpp" //recheck
#include <fstream>
#include "sensor_msgs/msg/camera_info.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/types.hpp>
#include <sstream>
#include "geometry_msgs/msg/pose.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Quaternion.h"
#include "whycon_ros.hpp"

whycon::WhyConROS::WhyConROS() : Node("whycon"), is_tracking(false), should_reset(true),node_handle(std::shared_ptr<WhyConROS>(this, [](auto *) {})), it(node_handle) 

{


  this->declare_parameter("targets", 1);
  this->declare_parameter("name", "default");
  this->declare_parameter("world_frame", "default");
  this->declare_parameter("outer_diameter", outer_diameter);
	this->declare_parameter("inner_diameter", inner_diameter);
  

  if (!this->get_parameter("targets", targets)) throw std::runtime_error("Private parameter \"targets\" is missing");
  
  this->get_parameter_or("name", frame_id, std::string("whycon"));
	this->get_parameter_or("world_frame", world_frame_id, std::string("world"));
  this->get_parameter_or("max_attempts", max_attempts, 1);
  this->get_parameter_or("max_refine", max_refine, 1);

	this->get_parameter("outer_diameter", parameters.outer_diameter);
	this->get_parameter("inner_diameter", parameters.inner_diameter);
	this->get_parameter("center_distance_tolerance_abs", parameters.center_distance_tolerance_abs);
	this->get_parameter("center_distance_tolerance_ratio", parameters.center_distance_tolerance_ratio);
	this->get_parameter("roundness_tolerance", parameters.roundness_tolerance);
	this->get_parameter("circularity_tolerance", parameters.circularity_tolerance);
	this->get_parameter("max_size", parameters.max_size);
	this->get_parameter("min_size", parameters.min_size);
	this->get_parameter("ratio_tolerance", parameters.ratio_tolerance);
	this->get_parameter("max_eccentricity", parameters.max_eccentricity);


  /* initialize ros */
  int input_queue_size = 1;
  this->get_parameter_or("input_queue_size", input_queue_size, input_queue_size);
  this->declare_parameter<std::string>("image_transport", "compressed");
  cam_sub = it.subscribeCamera("/image_raw", input_queue_size, std::bind(&WhyConROS::on_image, this, std::placeholders::_1, std::placeholders::_2));  //check again
  

  image_pub = it.advertise("/whycon/image_out", 1);
  poses_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("/whycon/poses", 1);
  context_pub = it.advertise("/whycon/context", 1);


  reset_service = this->create_service<std_srvs::srv::Empty>("reset", std::bind(&WhyConROS::reset, this, std::placeholders::_1, std::placeholders::_2));

}

void whycon::WhyConROS::on_image(const sensor_msgs::msg::Image::ConstSharedPtr &image_msg, const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info_msg)
{
   camera_model.fromCameraInfo(info_msg);
   if (camera_model.fullResolution().width == 0) { RCLCPP_ERROR_STREAM(this->get_logger(), "camera is not calibrated!"); return; }

  cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(image_msg, "rgb8");
  const cv::Mat& image = cv_ptr->image;

  if (!system)
    system = std::make_shared<whycon::LocalizationSystem>(targets, image.size().width, image.size().height, cv::Mat(camera_model.fullIntrinsicMatrix()), cv::Mat(camera_model.distortionCoeffs()), parameters);

  is_tracking = system->localize(image, should_reset/*!is_tracking*/, max_attempts, max_refine);

  if (is_tracking) {
    publish_results(image_msg->header, cv_ptr);
    should_reset = false;
  }
  
  else if (image_pub.getNumSubscribers() != 0)
    image_pub.publish(image_msg);

  if (context_pub.getNumSubscribers() != 0) {
    cv_bridge::CvImage cv_img_context;
    cv_img_context.encoding = cv_ptr->encoding;
    cv_img_context.header.stamp = cv_ptr->header.stamp;
    system->detector.context.debug_buffer(cv_ptr->image, cv_img_context.image);
    context_pub.publish((cv_img_context.toImageMsg()));
  }
}

void whycon::WhyConROS::reset(const std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  should_reset = true;
}

void whycon::WhyConROS::publish_results(const std_msgs::msg::Header& header, const cv_bridge::CvImageConstPtr& cv_ptr)
{
  bool publish_images = (image_pub.getNumSubscribers() != 0);
  bool publish_poses = (poses_pub->get_subscription_count() != 0);
  
  if (!publish_images && !publish_poses) return;
  
  // prepare image outpu
  cv::Mat output_image;
  if (publish_images)
    output_image = cv_ptr->image.clone();

  geometry_msgs::msg::PoseArray pose_array;
  
  // go through detected targets
  for (int i = 0; i < system->targets; i++) {
    const whycon::CircleDetector::Circle& circle = system->get_circle(i);
    whycon::LocalizationSystem::Pose pose = system->get_pose(circle);
    cv::Vec3f coord = pose.pos;

    // draw each target
    if (publish_images) {
      std::ostringstream ostr;
      ostr << std::fixed << std::setprecision(2);
			ostr << coord << " " << i;
      circle.draw(output_image, ostr.str(), cv::Vec3b(0,255,255));
			/*whycon::CircleDetector::Circle new_circle = circle.improveEllipse(cv_ptr->image);
			new_circle.draw(output_image, ostr.str(), cv::Vec3b(0,255,0));*/
			//cv::circle(output_image, camera_model.project3dToPixel(cv::Point3d(coord)), 1, cv::Scalar(255,0,255), 1, CV_AA);
      // The error you're encountering in ROS 2 when using CV_AA is because CV_AA is not defined in OpenCV for ROS 2. In OpenCV for ROS 2, anti-aliased drawing is typically achieved by using the LINE_AA flag.
      //You should replace CV_AA with LINE_AA like this:
      cv::circle(output_image, camera_model.project3dToPixel(cv::Point3d(coord)), 1, cv::Scalar(255, 0, 255), 1, cv::LINE_AA);

    }

    if (publish_poses) {
      geometry_msgs::msg::Pose p;
      p.position.x = pose.pos(0);
      p.position.y = pose.pos(1);
      p.position.z = pose.pos(2);

      tf2::Quaternion tf2_quat;
      tf2_quat.setRPY(0, pose.rot(0), pose.rot(1));
      geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(tf2_quat);
      p.orientation = msg_quat;
      pose_array.poses.push_back(p);
    }
  }

  if (publish_images) {
    cv_bridge::CvImage output_image_bridge = *cv_ptr;
    output_image_bridge.image = output_image;
    image_pub.publish(output_image_bridge.toImageMsg());
  }

  if (publish_poses) {
    pose_array.header = header;
    pose_array.header.frame_id = frame_id;
    poses_pub->publish(pose_array);
  }


}

