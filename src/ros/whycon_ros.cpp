#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/tf.h>
#include <sstream>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include "whycon_ros.h"
#include "whycon/PointArray.h"

whycon::WhyConROS::WhyConROS(ros::NodeHandle& n) : is_tracking(false), should_reset(true), it(n)
{
  if (!n.getParam("targets", targets)) throw std::runtime_error("Private parameter \"targets\" is missing");

  n.param("axis", axis_file, std::string());
  n.param("outer_diameter", outer_diameter, WHYCON_DEFAULT_OUTER_DIAMETER);
  n.param("inner_diameter", inner_diameter, WHYCON_DEFAULT_INNER_DIAMETER);
  n.param("max_attempts", max_attempts, 1);
  n.param("max_refine", max_refine, 1);
  n.param("xscale", xscale, 1.0);
  n.param("yscale", yscale, 1.0);

  /* initialize ros */
  int input_queue_size = 1;
  n.param("input_queue_size", input_queue_size, input_queue_size);
  cam_sub = it.subscribeCamera("/camera/image_rect_color", input_queue_size, boost::bind(&WhyConROS::on_image, this, _1, _2));
  
  image_pub = n.advertise<sensor_msgs::Image>("image_out", 1);
  viz_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  points_pub = n.advertise<whycon::PointArray>("points", 1);
  poses_pub = n.advertise<geometry_msgs::PoseArray>("poses", 1);
  trans_poses_pub = n.advertise<geometry_msgs::PoseArray>("trans_poses", 1);
  context_pub = n.advertise<sensor_msgs::Image>("context", 1);

  reset_service = n.advertiseService("reset", &WhyConROS::reset, this);
}

void whycon::WhyConROS::on_image(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  camera_model.fromCameraInfo(info_msg);
  if (camera_model.fullResolution().width == 0) { ROS_ERROR_STREAM("camera is not calibrated!"); return; }

  cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(image_msg, "rgb8");
  const cv::Mat& image = cv_ptr->image;

  if (!system) {
    system = boost::make_shared<cv::LocalizationSystem>(targets, image.size().width, image.size().height, cv::Mat(camera_model.fullIntrinsicMatrix()), cv::Mat(camera_model.distortionCoeffs()), outer_diameter, inner_diameter);
    if (!axis_file.empty()) {
      ROS_INFO_STREAM("opening axis file: " << axis_file + ".yml");
      system->read_axis(axis_file + ".yml", xscale, yscale);
    }
  }

  is_tracking = system->localize(image, should_reset/*!is_tracking*/, max_attempts, max_refine);

  if (is_tracking) {
    publish_results(image_msg->header, cv_ptr);
    should_reset = false;
  }
  else if (image_pub.getNumSubscribers() != 0)
    image_pub.publish(cv_ptr);

  if (context_pub.getNumSubscribers() != 0) {
    cv_bridge::CvImage cv_img_context;
    cv_img_context.encoding = cv_ptr->encoding;
    cv_img_context.header.stamp = cv_ptr->header.stamp;
    system->detector.context.debug_buffer(cv_ptr->image, cv_img_context.image);
    context_pub.publish(cv_img_context.toImageMsg());
  }
}

bool whycon::WhyConROS::reset(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  should_reset = true;
  return true;
}

void whycon::WhyConROS::publish_results(const std_msgs::Header& header, const cv_bridge::CvImageConstPtr& cv_ptr)
{
  bool publish_images = (image_pub.getNumSubscribers() != 0);
  bool publish_viz = (viz_pub.getNumSubscribers() != 0);
  bool publish_points = (points_pub.getNumSubscribers() != 0);
  bool publish_poses = (poses_pub.getNumSubscribers() != 0);
  bool publish_trans_poses = (trans_poses_pub.getNumSubscribers() != 0);
  
  if (!publish_images && !publish_viz && !publish_points && !publish_poses && !publish_trans_poses) return;
  
  // prepare particle markers
  visualization_msgs::Marker marker, transformed_marker;
  if (publish_viz)
  {
    marker.header = header;
    marker.header.frame_id = "/base_link";

    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.color.a = 1;
    marker.id = 0;
    
    transformed_marker = marker;

    marker.ns = "points";
    marker.ns = "poses";
    
    marker.color.r = 1;
    marker.color.g = 0;
    marker.color.b = 0;

    transformed_marker.color.r = 0;
    transformed_marker.color.g = 1;
    transformed_marker.color.b = 0;
  }

  // prepare image outpu
  cv::Mat output_image;
  if (publish_images)
    output_image = cv_ptr->image.clone();

  geometry_msgs::PoseArray pose_array, trans_pose_array;
  whycon::PointArray point_array;
  
  // go through detected targets
  for (int i = 0; i < system->targets; i++) {
    const cv::CircleDetector::Circle& circle = system->get_circle(i);
    cv::LocalizationSystem::Pose pose = system->get_pose(circle);
    cv::LocalizationSystem::Pose trans_pose = system->get_transformed_pose(circle);
    cv::Vec3f coord = pose.pos;    
    cv::Vec3f coord_trans = trans_pose.pos;

    // draw each target
    if (publish_images) {
      std::ostringstream ostr;
      ostr << std::fixed << std::setprecision(2);
      ostr << coord_trans << " " << i;
      circle.draw(output_image, ostr.str(), cv::Vec3b(0,255,255));
    }

    if (publish_viz) {
      geometry_msgs::Point marker_point;
      marker_point.x = coord(0);
      marker_point.y = coord(1);
      marker_point.z = coord(2);  
      marker.points.push_back(marker_point);
      if (system->axis_set) {
        marker_point.x = coord_trans(0);
        marker_point.y = coord_trans(1);
        marker_point.z = coord_trans(2);  
        transformed_marker.points.push_back(marker_point);
      }
    }

    if (publish_trans_poses) {
      geometry_msgs::Pose p;
      p.position.x = trans_pose.pos(0);
      p.position.y = trans_pose.pos(1);
      p.position.z = trans_pose.pos(2);
      p.orientation = tf::createQuaternionMsgFromRollPitchYaw(trans_pose.rot(0), trans_pose.rot(1), trans_pose.rot(2));
      trans_pose_array.poses.push_back(p);
    }

    if (publish_poses) {
      geometry_msgs::Pose p;
      p.position.x = pose.pos(0);
      p.position.y = pose.pos(1);
      p.position.z = pose.pos(2);
      p.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, pose.rot(0), pose.rot(1));
      pose_array.poses.push_back(p);
    }

    if (publish_points) {
      geometry_msgs::Point p;
      p.x = circle.x;
      p.y = circle.y;
      p.z = 0;
      point_array.points.push_back(p);
    }
  }

  if (publish_viz) {
    viz_pub.publish(marker);
    if (system->axis_set) viz_pub.publish(transformed_marker);
  }
  
  if (publish_images) {
    cv_bridge::CvImage output_image_bridge = *cv_ptr;
    output_image_bridge.image = output_image;
    image_pub.publish(output_image_bridge);
  }

  if (publish_poses) {
    pose_array.header = header;
    pose_array.header.frame_id = "/base_link";
    poses_pub.publish(pose_array);
  }

  if (publish_trans_poses) {
    trans_pose_array.header = header;
    trans_pose_array.header.frame_id = "/base_link";
    trans_poses_pub.publish(trans_pose_array);
  }

  if (publish_points) {
    point_array.header = header;
    point_array.header.frame_id = "/base_link";
    points_pub.publish(point_array);
  }
}




