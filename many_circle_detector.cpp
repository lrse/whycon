#include <iostream>
#include "many_circle_detector.h"
using namespace std;

cv::ManyCircleDetector::ManyCircleDetector(int _number_of_circles, int _width, int _height, float _diameter_ratio) : 
  context(_width, _height), width(_width), height(_height), number_of_circles(_number_of_circles)
{
  circles.resize(number_of_circles);
  detectors.resize(number_of_circles, CircleDetector(width, height, &context, _diameter_ratio));
}

cv::ManyCircleDetector::~ManyCircleDetector(void) {
}

bool cv::ManyCircleDetector::initialize(const cv::Mat& image) {
  cv::Mat marked_image;
  image.copyTo(marked_image);

  /*cv::namedWindow("marked", CV_WINDOW_NORMAL);
  cv::namedWindow("buffer", CV_WINDOW_NORMAL);*/
  int attempts = 100;
  for (int i = 0; i < number_of_circles; i++) {
    detectors[i].draw = true;
    for (int j = 0; j < attempts; j++) {
      int64_t ticks = cv::getTickCount();
      cout << "detecting circle " << i << " attempt " << j << endl;
      circles[i] = detectors[i].detect(marked_image, (i != 0 ? circles[i - 1] : CircleDetector::Circle()));
      /*cv::imshow("marked", marked_image);
      cv::Mat buffer_img;
      cv::waitKey();*/
      double delta = (double)(cv::getTickCount() - ticks) / cv::getTickFrequency();
      cout << "t: " << delta << " " << " fps: " << 1/delta << endl;
      if (circles[i].valid) break;
    }
    detectors[i].draw = false;
    
    if (!circles[i].valid) return false;    
  }
  return true;
}

bool cv::ManyCircleDetector::detect(const cv::Mat& image, int refine_max_step) {
  bool all_detected = true;
  cv::Mat marked_image;
  image.copyTo(marked_image);

  for (int i = 0; i < number_of_circles && all_detected; i++) {
    cout << "detecting circle " << i << endl;

    for (int refine_counter = 0; refine_counter < refine_max_step; refine_counter++)
    {
      if (refine_counter > 0) cout << "refining step " << refine_counter << "/" << refine_max_step << endl;
      
      int prev_threshold = detectors[i].get_threshold();     
      int64_t ticks = cv::getTickCount();    
      circles[i] = detectors[i].detect(marked_image, circles[i]); // TODO: modify current
      if (!circles[i].valid) { all_detected = false; break; }
      double delta = (double)(cv::getTickCount() - ticks) / cv::getTickFrequency();
      cout << "t: " << delta << " " << " fps: " << 1/delta << endl;

      // if the threshold changed, keep refining this circle
      if (detectors[i].get_threshold() == prev_threshold) break;
    }
  }
  
  return all_detected;
}
