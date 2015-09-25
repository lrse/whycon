#include "set_axis.h"

whycon::AxisSetter::AxisSetter(ros::NodeHandle &n) : it(n)
{
  set_axis_now = is_tracking = false;
  cam_sub = it.subscribeCamera("/camera/image_rect_color", 1, boost::bind(&AxisSetter::on_image, this, _1, _2));
  img_pub = n.advertise<sensor_msgs::Image>("image_out", 1);

  if (!n.getParam("axis", axis_name)) throw std::runtime_error("Please specify the name of the axis to be saved");
  n.param("outer_diameter", outer_diameter, WHYCON_DEFAULT_OUTER_DIAMETER);
  n.param("inner_diameter", inner_diameter, WHYCON_DEFAULT_INNER_DIAMETER);

  set_axis_service = n.advertiseService("set", &AxisSetter::set_axis, this);
}

bool whycon::AxisSetter::set_axis(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  set_axis_now = true;
  return true;
}


void whycon::AxisSetter::on_image(const sensor_msgs::ImageConstPtr& img_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  camera_model.fromCameraInfo(info_msg);
  if (camera_model.fullResolution().width == 0) { ROS_ERROR_STREAM("camera is not calibrated!"); return; }

  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img_msg);
  cv::Mat& image = cv_ptr->image;

  if (!system)
    system = boost::make_shared<cv::LocalizationSystem>(4, image.size().width, image.size().height, cv::Mat(camera_model.fullIntrinsicMatrix()), cv::Mat(camera_model.distortionCoeffs()), outer_diameter, inner_diameter);

  if (set_axis_now) {
		if (!system->set_axis(image, 5, 5, axis_name + ".yml"))
			ROS_ERROR_STREAM("could not set axis, attempting again");
		else
			set_axis_now = false;
  }

  if (system->axis_set) {
    system->draw_axis(image);
  }
  else {
    is_tracking = system->localize(image, !is_tracking, 5, 1);
    for (int i = 0; i < system->targets; i++) {
      const cv::CircleDetector::Circle& circle = system->get_circle(i);
      if (!circle.valid) continue;

      //cv::LocalizationSystem::Pose pose = system->get_pose(circle);
      cv::LocalizationSystem::Pose trans_pose = system->get_transformed_pose(circle);
      //cv::Vec3f coord = pose.pos;
      cv::Vec3f coord_trans = trans_pose.pos;

      // draw each target
      std::ostringstream ostr;
      ostr << std::fixed << std::setprecision(2);
      ostr << coord_trans << " " << i;
      circle.draw(image, ostr.str(), cv::Vec3b(0,255,255));
    }
  }
  img_pub.publish(cv_ptr);
}
