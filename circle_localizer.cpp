#include <iostream>
#include "circle_localizer.h"
using namespace std;

cv::CircleLocalizer::CircleLocalizer(int _number_of_circles, int _width, int _height) : stop(false),
  start_barrier(_number_of_circles + 1), end_barrier(_number_of_circles + 1), width(_width), height(_height), 
  number_of_circles(_number_of_circles)
{
  circles.resize(number_of_circles);
  detectors.resize(number_of_circles, CircleDetector(width, height));
  
  /*threads.resize(number_of_circles);
  for (int i = 0; i < number_of_circles; i++)
    threads[i] = boost::shared_ptr<boost::thread>(new boost::thread(&CircleLocalizer::localize_individual, this, i));*/
}

cv::CircleLocalizer::~CircleLocalizer(void) {
  stop = true;
  
  /*start_barrier.wait(); // wake up all threads, so they abort their loop
  for (int i = 0; i < number_of_circles; i++)
    threads[i]->join();*/
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

void cv::CircleLocalizer::localize(const cv::Mat& image) {
  image.copyTo(current_image);
  
  /*start_barrier.wait(); // wakes up all threads
  end_barrier.wait(); // blocks until all threads finish*/
  
  /*for (int i = 0; i < number_of_circles; i++) {
    //int64_t ticks = cv::getTickCount();
    circles[i] = detectors[i].detect(current_image, circles[i]); // TODO: modify current
    //double delta = (double)(cv::getTickCount() - ticks) / cv::getTickFrequency();
    //cout << "tinner: " << delta << " " << " fps: " << 1/delta << endl;
  }*/
  
  static tbb::affinity_partitioner ap;
  tbb::parallel_for(tbb::blocked_range<int>(0, number_of_circles, 4), Functor(*this), ap);
}

void cv::CircleLocalizer::localize_individual(int id) {  
  CircleDetector::Circle& circle = circles[id];
  CircleDetector& detector = detectors[id];
  
  cv::Mat test;
  current_image.copyTo(test);
  while(true) {
    start_barrier.wait();
    if (stop) break;
    current_image.copyTo(test);
    int64_t ticks = cv::getTickCount();
    circle = detector.detect(test, circle); // TODO: modify current
    double delta = (double)(cv::getTickCount() - ticks) / cv::getTickFrequency();
    cout << "tinner: " << delta << " " << " fps: " << 1/delta << endl;
    
    end_barrier.wait();
  }
}

cv::CircleLocalizer::Functor::Functor(cv::CircleLocalizer& _localizer) : image(_localizer.current_image), localizer(_localizer), detectors(_localizer.detectors), circles(_localizer.circles)
{
}

void cv::CircleLocalizer::Functor::operator()(tbb::blocked_range<int>& r) const
{
  for (uint i = r.begin(); i != r.end(); i++) {
    //int64_t ticks = cv::getTickCount();
    circles[i] = detectors[i].detect(image, circles[i]); // TODO: modify current
    //double delta = (double)(cv::getTickCount() - ticks) / cv::getTickFrequency();
    //cout << "tinner: " << delta << " " << " fps: " << 1/delta << endl;
  }
}
