#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <signal.h>
#include "circle_detector.h"
using namespace std;

bool stop = false;
void interrupt(int s) {
  stop = true;
}

int main(int argc, char** argv) {
  if (argc < 3) {
    cout << "usage: detector <-img file|-video file|-cam camera-number>" << endl;
    return 1;
  }
  
  string mode(argv[1]);
  if (mode == "-img") {
    cv::Mat img = cv::imread(argv[2]);
    cv::CircleDetector detector(img.size().width, img.size().height);
    
    cv::CircleDetector::Circle previous_circle;
    cv::Mat input_img;
    while (!stop) {
      img.copyTo(input_img);
      int64_t ticks = cv::getTickCount();
      previous_circle = detector.detect(input_img, previous_circle);
      double delta = (double)(cv::getTickCount() - ticks) / cv::getTickFrequency();
      cout << "t: " << delta << " " << " fps: " << 1/delta << endl;
      
      if (previous_circle.valid) cv::circle(input_img, cv::Point(previous_circle.x, previous_circle.y), previous_circle.size / 2, cv::Scalar(0,255,0,128), -1);
      /*cv::imshow("result", input_img);
      if (cv::waitKey() == 27) break;*/
    }
  }
  else if (mode == "-video" || mode == "-cam")
  {
    bool is_video = (mode == "-video");
    cv::VideoCapture capture;
    if (is_video) capture.open(argv[2]);
    else capture.open(atoi(argv[2]));
    
    if (!capture.isOpened()) { cout << "Could not open file/device " << argv[2] << endl; return 1; }
    
    int width = capture.get(CV_CAP_PROP_FRAME_WIDTH);
    int height = capture.get(CV_CAP_PROP_FRAME_HEIGHT);
    cv::CircleDetector detector(width, height);
    
    cvStartWindowThread();
    cv::namedWindow("result");
    
    cv::Mat img;
    cv::CircleDetector::Circle previous_circle;
    while (!stop) {
      if (!capture.read(img)) break;
      
      int64_t ticks = cv::getTickCount();
      previous_circle = detector.detect(img, previous_circle);
      double delta = (double)(cv::getTickCount() - ticks) / cv::getTickFrequency();
      cout << "t: " << delta << " " << " fps: " << 1/delta << endl;
      cout << previous_circle.x << " " << previous_circle.y << endl;
      cv::imshow("result", img);
    }
  }
  else {
    cout << "Unknown mode '" << argv[1] << "'" << endl;
    return 1;
  }
  return 0;
}
