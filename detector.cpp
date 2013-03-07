#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include "circle_detector.h"
using namespace std;

int main(int argc, char** argv) {
  if (argc < 3) {
    cout << "usage: detector <-img file|-video file|-cam camera-number w h>" << endl;
    return 1;
  }
  
  
  if (string(argv[1]) == "-img") {
    cv::Mat img = cv::imread(argv[2]);
    cv::CircleDetector detector(img.size().width, img.size().height);
    
    cv::CircleDetector::Circle previous_circle;
    cv::Mat input_img;
    while (true) {
      img.copyTo(input_img);
      int64_t ticks = cv::getTickCount();
      previous_circle = detector.detect(input_img, previous_circle);
      double delta = (double)(cv::getTickCount() - ticks) / cv::getTickFrequency();
      cout << "t: " << delta << " " << " fps: " << 1/delta << endl;
      //if (previous_circle.valid) cv::circle(input_img, cv::Point(previous_circle.x, previous_circle.y), previous_circle.size / 2, cv::Scalar(0,255,0,128), -1);
      //cv::imshow("result", input_img);
      //if (cv::waitKey() == 27) break;
    }
  }
  else if (string(argv[1]) == "-video") {
  }
  else if (string(argv[1]) == "-cam") {
  }
  else {
    cout << "Unknown mode '" << argv[1] << "'" << endl;
    return 1;
  }
  return 0;
}
