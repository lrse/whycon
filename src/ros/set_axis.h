#ifndef SET_AXIS_NODE_H
#define SET_AXIS_NODE_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseArray.h>
#include <yaml-cpp/yaml.h>
#include <sensor_msgs/Image.h>

namespace whycon {
  class AxisSetter {
    public:
      AxisSetter(ros::NodeHandle& n);

			ros::Subscriber poses_sub, image_sub;
			ros::Publisher image_pub;

      void on_poses(const geometry_msgs::PoseArrayConstPtr& poses_msg);
			void on_image(const sensor_msgs::ImageConstPtr& image_msg);

			double xscale, yscale;
			bool transforms_set;

		private:
			void detect_square(const std::vector<geometry_msgs::Pose>& msg_poses, std::vector<tf::Point>& points);
			tf::Matrix3x3 compute_projection(const std::vector<tf::Point>& points, float xscale, float yscale);
			tf::Transform compute_similarity(const std::vector<tf::Point>& points);

			void write_projection(YAML::Emitter& yaml, const tf::Matrix3x3& projection);
			void write_similarity(YAML::Emitter& yaml, const tf::Transform& similarity);
	};
}

#endif // SET_AXIS_NODE_H
