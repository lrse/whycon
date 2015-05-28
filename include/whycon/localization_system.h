#ifndef __LOCALIZATION_SYSYEM__
#define __LOCALIZATION_SYSYEM__

#include <opencv2/opencv.hpp>
#include <whycon/many_circle_detector.h>

namespace cv {
  class LocalizationSystem {
    public:
      LocalizationSystem(int targets, int width, int height, const cv::Mat& K, const cv::Mat& dist_coeff,
        float outer_diameter = WHYCON_DEFAULT_OUTER_DIAMETER, float inner_diameter = WHYCON_DEFAULT_INNER_DIAMETER);
      
      bool set_axis(const cv::Mat& image, int attempts = 1, int max_refine = 1, const std::string& output = std::string());
      void read_axis(const std::string& input, float xscale, float yscale);
      void draw_axis(cv::Mat& image);
      
      bool localize(const cv::Mat& image, bool reset = false, int attempts = 1, int max_refine = 1);
      
      float xscale, yscale;

      // TODO: use double?
      struct Pose {
        cv::Vec3f pos;
        cv::Vec3f rot; // pitch, roll, yaw
      };
      
      Pose get_pose(int id) const;
      Pose get_pose(const CircleDetector::Circle& circle) const;
      const CircleDetector::Circle& get_circle(int id);
      
      Pose get_transformed_pose(int id) const;
      Pose get_transformed_pose(const CircleDetector::Circle& circle) const;
      
      static void load_matlab_calibration(const std::string& calib_file, cv::Mat& K, cv::Mat& dist_coeff);
      static void load_opencv_calibration(const std::string& calib_file, cv::Mat& K, cv::Mat& dist_coeff);
      
      CircleDetector::Circle origin_circles[4]; // center, X, Y
      
      cv::Matx33f coordinates_transform;      
      ManyCircleDetector detector;
      
      int targets, width, height;
      bool axis_set;
      
    private:
      cv::Mat K, dist_coeff;
      float circle_diameter;
      double fc[2]; // focal length X,Y
      double cc[2]; // principal point X,Y
      double kc[6]; // distortion coefficients
      void transform(double x_in, double y_in, double& x_out, double& y_out) const;

      void precompute_undistort_map(void);
      cv::Mat undistort_map;
  };
}

#endif
