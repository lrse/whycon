#ifndef __LOCALIZATION_SYSYEM__
#define __LOCALIZATION_SYSYEM__

#include <opencv2/opencv.hpp>
#include "many_circle_detector.h"

namespace cv {
  class LocalizationSystem {
    public:
      LocalizationSystem(int targets, int width, int height, const cv::Mat& K, const cv::Mat& dist_coeff,
        float diameter = 0.122, float diameter_ratio = WHYCON_DEFAULT_DIAMETER_RATIO);
      
      bool set_axis(const cv::Mat& image);
      void draw_axis(cv::Mat& image);
      
      bool initialize(const cv::Mat& image);
      bool localize(const cv::Mat& image, int attempts = 1);
      
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
      
    private:
      cv::Mat K, dist_coeff;
      float circle_diameter;
      double fc[2]; // focal length X,Y
      double cc[2]; // principal point X,Y
      double kc[6]; // distortion coefficients
      float unbarrel_x(float x, float y) const;
      float unbarrel_y(float x, float y) const;
      float transform_x(float xc,float yc) const;
      float transform_y(float xc,float yc) const;
      void transform(float x, float y) const;
      
      cv::Vec3f eigen(double data[]) const;        
  };
}

#endif
