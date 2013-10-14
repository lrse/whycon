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

bool cv::ManyCircleDetector::detect(const cv::Mat& image, bool reset, int max_attempts, int refine_max_step) {
  bool all_detected = true;

  cv::Mat input;
  if (reset) image.copyTo(input); // image will be modified on reset
  else input = image;
  
  for (int i = 0; i < number_of_circles && all_detected; i++) {    
    cout << "detecting circle " << i << endl;
    
    for (int j = 0; j < max_attempts; j++) {
      cout << "attempt " << j << endl;

      for (int refine_counter = 0; refine_counter < refine_max_step; refine_counter++)
      {
        if (refine_counter > 0) cout << "refining step " << refine_counter << "/" << refine_max_step << endl;
        int prev_threshold = detectors[i].get_threshold();

        int64_t ticks = cv::getTickCount();
        
        if (refine_counter == 0 && reset)
          circles[i] = detectors[i].detect(input, (i == 0 ? CircleDetector::Circle() : circles[i-1]));
        else
          circles[i] = detectors[i].detect(input, circles[i]);
          
        double delta = (double)(cv::getTickCount() - ticks) / cv::getTickFrequency();
        cout << "t: " << delta << " " << " fps: " << 1/delta << endl;

        if (!circles[i].valid) break;

        // if the threshold changed, keep refining this circle
        if (detectors[i].get_threshold() == prev_threshold) break;
      }

      if (circles[i].valid) {
        cout << "detection of circle " << i << " ok" << endl;
        if (reset) detectors[i].cover_last_detected(input);
        break; // detection was successful, dont keep trying
      }
    }

    // detection was not possible for this circle, abort search
    if (!circles[i].valid) { all_detected = false; break; }
  }
  
  return all_detected;
}
