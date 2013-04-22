#ifndef __LOCALIZATION_SYSYEM__
#define __LOCALIZATION_SYSYEM__

#include <opencv2/opencv.hpp>
#include "circle_localizer.h"
#include "config.h"

namespace cv {
  class LocalizationSystem {
    public:
      // current diameter = 0.123
      LocalizationSystem(int targets, int width, int height, const cv::Mat& K, const cv::Mat& dist_coeff, float diameter);
      
      bool set_axis(const cv::Mat& image);
      void draw_axis(cv::Mat& image);
      
      bool initialize(const cv::Mat& image);
      bool localize(const cv::Mat& image, int attempts = 1);
      
      float xscale, yscale;
      
      struct Pose {
        cv::Vec3f pos;
        cv::Vec3f rot; // pitch, roll, yaw
      };
      
      Pose get_pose(int id);
      Pose get_pose(const CircleDetector::Circle& circle);
      const CircleDetector::Circle& get_circle(int id);
      
      Pose get_transformed_pose(int id);
      Pose get_transformed_pose(const CircleDetector::Circle& circle);
      
      static void load_matlab_calibration(const std::string& calib_file, cv::Mat& K, cv::Mat& dist_coeff);
      static void load_opencv_calibration(const std::string& calib_file, cv::Mat& K, cv::Mat& dist_coeff);
      
      CircleDetector::Circle origin_circles[3]; // center, X, Y
      
      #ifdef ENABLE_PROJECTIVITY
      cv::Matx23f coordinates_transform;
      #else
      cv::Matx33f coordinates_transform;
      #endif
      
      CircleLocalizer localizer;
      
    private:
      int targets, width, height;
      
      
      cv::Mat K, dist_coeff;
      float circle_diameter;
      double fc[2]; // focal length X,Y
      double cc[2]; // principal point X,Y
      double kc[6]; // distortion coefficients
      float unbarrel_x(float x, float y);
      float unbarrel_y(float x, float y);
      float transform_x(float xc,float yc);
      float transform_y(float xc,float yc);
      void transform(float x, float y);
      
      cv::Vec3f eigen(double data[]);        
  };
}

#endif
