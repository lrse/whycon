#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <signal.h>
#include "many_circle_detector.h"
using namespace std;

bool stop = false;
void interrupt(int s) {
  stop = true;
}

int main(int argc, char** argv) {
  if (argc < 4) {
    cout << "usage: test-localizer <number of circles> <-img file|-video file|-cam camera-number>" << endl;
    return 1;
  }
  
  int number_of_circles = atoi(argv[1]);
  cout << "Tracking " << number_of_circles << " circles" << endl;
  string mode(argv[2]);
  if (mode == "-img") {
    //cvStartWindowThread();
    //cv::namedWindow("results");
    cv::Mat img = cv::imread(argv[3]);
    cv::ManyCircleDetector detector(number_of_circles, img.size().width, img.size().height);
    
    int64_t ticks = cv::getTickCount();
    bool initialized = detector.initialize(img);
    double delta = (double)(cv::getTickCount() - ticks) / cv::getTickFrequency();
    cout << "t: " << delta << " " << " fps: " << 1/delta << " [initialize]" << endl;
    if (!initialized) {
      cout << "Not all circles were found" << endl;
      return 1;
    }
    
    cv::Mat input_img;
    while (!stop) {
      img.copyTo(input_img);
      //cout << "localizing" << endl;
      ticks = cv::getTickCount();
      detector.detect(input_img);
      delta = (double)(cv::getTickCount() - ticks) / cv::getTickFrequency();
      cout << "t: " << delta << " " << " fps: " << 1/delta << "[total]" << endl;
      
      /*for (int i = 0; i < number_of_circles; i++) {
        if (localizer.circles[i].valid) {
          cout << "circle at " << localizer.circles[i].x << " " << localizer.circles[i].y << endl;
          //cv::circle(input_img, cv::Point(localizer.circles[i].x, localizer.circles[i].y), 5, cv::Scalar(0,255,0,128), -1);
        }
      }*/
      /*cv::imshow("results", input_img);
      if (cv::waitKey() == 27) break;*/
    }
  }
  else if (mode == "-video" || mode == "-cam")
  {
    bool is_video = (mode == "-video");
    cv::VideoCapture capture;
    if (is_video) capture.open(argv[3]);
    else capture.open(atoi(argv[3]));
    
    if (!capture.isOpened()) { cout << "Could not open file/device " << argv[2] << endl; return 1; }
    
    int width = capture.get(CV_CAP_PROP_FRAME_WIDTH);
    int height = capture.get(CV_CAP_PROP_FRAME_HEIGHT);
    cv::ManyCircleDetector detector(number_of_circles, width, height);
    bool initialized = false;
    
    cvStartWindowThread();
    cv::namedWindow("result");
    
    cv::Mat img;
    cv::CircleDetector::Circle previous_circle;
    while (!stop) {
      if (!capture.read(img)) break;
      
      if (!initialized) {
        if (!detector.initialize(img)) { cout << "circles not detected" << endl; break; }
      }
      
      detector.detect(img);
      cv::imshow("result", img);
    }
  }
  else {
    cout << "Unknown mode '" << argv[1] << "'" << endl;
    return 1;
  }
  return 0;
}
