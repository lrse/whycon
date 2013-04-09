#include <iostream>
#include "circle_localizer.h"
using namespace std;

cv::CircleLocalizer::CircleLocalizer(int _number_of_circles, int _width, int _height) : 
  width(_width), height(_height), number_of_circles(_number_of_circles)
{
  circles.resize(number_of_circles);
  detectors.resize(number_of_circles, CircleDetector(width, height));
}

cv::CircleLocalizer::~CircleLocalizer(void) {
}

bool cv::CircleLocalizer::initialize(const cv::Mat& image) {
  cv::Mat marked_image;
  image.copyTo(marked_image);
  
  int attempts = 100;
  for (int i = 0; i < number_of_circles; i++) {
    detectors[i].draw = true;
    for (int j = 0; j < attempts; j++) {
      marked_image.copyTo(current_image);
      
      //cv::imshow("result", current_image);
      //cv::waitKey();
      
      circles[i] = detectors[i].detect(current_image);
      cout << "attempt " << j << " - circle valid? " << circles[i].valid << " " << circles[i].x << "," << circles[i].y << endl;
      
      //cv::imshow("result", current_image);
      //cv::waitKey();
      if (circles[i].valid) { current_image.copyTo(marked_image); break; }
    }
    detectors[i].draw = false;
    
    if (!circles[i].valid) return false;    
  }
  return true;
}

bool cv::CircleLocalizer::localize(const cv::Mat& image) {
  bool all_detected = true;
  for (int i = 0; i < number_of_circles; i++) {
    //int64_t ticks = cv::getTickCount();
    circles[i] = detectors[i].detect(image, circles[i]); // TODO: modify current
    if (!circles[i].valid) all_detected = false;
    //double delta = (double)(cv::getTickCount() - ticks) / cv::getTickFrequency();
    //cout << "tinner: " << delta << " " << " fps: " << 1/delta << endl;
  }
  return all_detected;
  
  /*static tbb::affinity_partitioner ap;
  tbb::parallel_for(tbb::blocked_range<int>(0, number_of_circles, 8), Functor(*this, image), ap);*/
}

cv::CircleLocalizer::Functor::Functor(cv::CircleLocalizer& _localizer, const cv::Mat& _image) : image(_image), localizer(_localizer), circles(_localizer.circles), detectors(_localizer.detectors)
{
}

void cv::CircleLocalizer::Functor::operator()(tbb::blocked_range<int>& r) const
{
  for (int i = r.begin(); i != r.end(); i++) {
    //int64_t ticks = cv::getTickCount();
    circles[i] = detectors[i].detect(image, circles[i]); // TODO: modify current
    //double delta = (double)(cv::getTickCount() - ticks) / cv::getTickFrequency();
    //cout << "tinner: " << delta << " " << " fps: " << 1/delta << endl;
  }
}
