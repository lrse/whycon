#include <iostream>
#include "many_circle_detector.h"
using namespace std;

cv::ManyCircleDetector::ManyCircleDetector(int _number_of_circles, int _width, int _height) : 
  width(_width), height(_height), number_of_circles(_number_of_circles)
{
  circles.resize(number_of_circles);
  detectors.resize(number_of_circles, CircleDetector(width, height));
}

cv::ManyCircleDetector::~ManyCircleDetector(void) {
}

bool cv::ManyCircleDetector::initialize(const cv::Mat& image) {
  cv::Mat marked_image, current_image;
  image.copyTo(marked_image);

  int attempts = 100;
  for (int i = 0; i < number_of_circles; i++) {
    detectors[i].draw = true;
    for (int j = 0; j < attempts; j++) {
      marked_image.copyTo(current_image);

      cout << "detecting circle " << i << " attempt " << j << endl;
      circles[i] = detectors[i].detect(current_image);
      if (circles[i].valid) { current_image.copyTo(marked_image); break; }
    }
    detectors[i].draw = false;
    
    if (!circles[i].valid) return false;    
  }
  return true;
}

bool cv::ManyCircleDetector::detect(const cv::Mat& image) {
  bool all_detected = true;

  for (int i = 0; i < number_of_circles; i++) {
    //int64_t ticks = cv::getTickCount();
    cout << "detecting circle " << i << endl;
    circles[i] = detectors[i].detect(image, circles[i]); // TODO: modify current
    if (!circles[i].valid) { all_detected = false; break; }
    //double delta = (double)(cv::getTickCount() - ticks) / cv::getTickFrequency();
    //cout << "tinner: " << delta << " " << " fps: " << 1/delta << endl;
  }
  return all_detected;
}

/*bool cv::ManyCircleDetector::localize_parallel(const cv::Mat& image) {
  static tbb::affinity_partitioner ap;
  tbb::parallel_for(tbb::blocked_range<int>(0, number_of_circles, 8), Functor(*this, image), ap);
  // TODO: check if all_detected
}*/

cv::ManyCircleDetector::Functor::Functor(cv::ManyCircleDetector& _detector, const cv::Mat& _image) : image(_image), detector(_detector), circles(_detector.circles), detectors(_detector.detectors)
{
}

void cv::ManyCircleDetector::Functor::operator()(tbb::blocked_range<int>& r) const
{
  for (int i = r.begin(); i != r.end(); i++) {
    //int64_t ticks = cv::getTickCount();
    circles[i] = detectors[i].detect(image, circles[i]); // TODO: modify current
    //double delta = (double)(cv::getTickCount() - ticks) / cv::getTickFrequency();
    //cout << "tinner: " << delta << " " << " fps: " << 1/delta << endl;
  }
}
