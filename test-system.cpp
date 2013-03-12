#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <signal.h>
#include "localization_system.h"
using namespace std;

bool stop = false;
void interrupt(int s) {
  stop = true;
}

bool clicked = false;
void mouse_callback(int event, int x, int y, int flags, void* param) {
  if (event == CV_EVENT_LBUTTONDOWN) clicked = true;
}

int main(int argc, char** argv) {
  if (argc < 3) {
    cout << "usage: test-localizer <number of circles> <camera-number>" << endl;
    return 1;
  }
  
  int number_of_circles = atoi(argv[1]);
  cv::VideoCapture capture(atoi(argv[2]));
  if (!capture.isOpened()) { cout << "error opening camera" << endl; return 1; }
  
  cv::Mat frame;
  cv::Mat K, dist_coeff;
  cv::LocalizationSystem::load_matlab_calibration("../Calib_Results.m", K, dist_coeff);
  cv::LocalizationSystem system(number_of_circles, capture.get(CV_CAP_PROP_FRAME_WIDTH), capture.get(CV_CAP_PROP_FRAME_HEIGHT),
    K, dist_coeff, 0.123);
  
  cvStartWindowThread();
  cv::namedWindow("output");
  cv::setMouseCallback("output", mouse_callback);
  bool axis_was_set = false;
  bool start_tracking = false;
  cv::Mat original_frame;
  while (true) {
    capture >> original_frame;
    original_frame.copyTo(frame);
    if (!axis_was_set && clicked) {
      system.set_axis(original_frame);
      clicked = false;
      axis_was_set = true;
    }
    if (axis_was_set) {
      cv::ellipse(frame, cv::Point(system.origin_circles[0].x, system.origin_circles[0].y), cv::Size((int)system.origin_circles[0].m0 * 2, (int)system.origin_circles[0].m1 * 2),
        atan2(system.origin_circles[0].v1, system.origin_circles[0].v0),
        0, 360, cv::Scalar(255,255,0), 2, CV_AA);
      cv::putText(frame, "center", cv::Point(system.origin_circles[0].x, system.origin_circles[0].y), CV_FONT_HERSHEY_SIMPLEX,
      1.0, cv::Scalar(255,0,0), 3);
      cv::ellipse(frame, cv::Point(system.origin_circles[1].x, system.origin_circles[1].y), cv::Size((int)system.origin_circles[1].m0 * 2, (int)system.origin_circles[1].m1 * 2),
        atan2(system.origin_circles[1].v1, system.origin_circles[1].v0),
        0, 360, cv::Scalar(255,255,0), 2, CV_AA);
      cv::putText(frame, "X", cv::Point(system.origin_circles[1].x, system.origin_circles[1].y), CV_FONT_HERSHEY_SIMPLEX,
      1.0, cv::Scalar(0,255,0), 3);
      cv::ellipse(frame, cv::Point(system.origin_circles[2].x, system.origin_circles[2].y), cv::Size((int)system.origin_circles[2].m0 * 2, (int)system.origin_circles[2].m1 * 2),
        atan2(system.origin_circles[2].v1, system.origin_circles[2].v0),
        0, 360, cv::Scalar(255,255,0), 2, CV_AA);
      cv::putText(frame, "Y", cv::Point(system.origin_circles[2].x, system.origin_circles[2].y), CV_FONT_HERSHEY_SIMPLEX,
      1.0, cv::Scalar(0,0,255), 3);
      
      cout << "center to X " << cv::norm(system.get_pose(system.origin_circles[0]).pos, system.get_pose(system.origin_circles[1]).pos) << endl;
      cout << "center to Y " << cv::norm(system.get_pose(system.origin_circles[0]).pos, system.get_pose(system.origin_circles[2]).pos) << endl;
      cout << "X to Y " << cv::norm(system.get_pose(system.origin_circles[1]).pos, system.get_pose(system.origin_circles[2]).pos) << endl;
      
      if (clicked) {
        clicked = false;
        start_tracking = true;
        system.initialize(original_frame);
      }
    }
    
    if (start_tracking) {
      system.localize(original_frame);
      cv::Vec2f coord = system.coordinates_transform * system.get_pose(0).pos;
      cv::circle(frame, cv::Point(system.localizer.circles[0].x, system.localizer.circles[0].y), 3, cv::Scalar(255,255,0), -1);
      ostringstream ostr;
      ostr << coord;
      cv::putText(frame, ostr.str(), cv::Point(system.localizer.circles[0].x, system.localizer.circles[0].y), CV_FONT_HERSHEY_SIMPLEX,
      0.8, cv::Scalar(0,0,0), 3);
    }
    cv::imshow("output", frame);
  }
  return 0;
}

