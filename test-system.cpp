#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <signal.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
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
  if (argc < 5) {
    cout << "usage: test-localizer <number of circles> <camera-number> <output name> <unit scale>" << endl;
    cout << " An output directory will be created with the specified name, where frames will be saved," << endl;
    cout << " along the corresponding information (circle positions, scale)" << endl;
    return 1;
  }
  
  /* create output directory */
  string output_directory(argv[3]);
  if (mkdir(output_directory.c_str(), 0777) == -1) { cerr << "Error creating output directory '" << output_directory << "': " << strerror(errno) << endl; return 1; }
  
  float unit_scale = atof(argv[4]);
  
  /* setup camera */
  cv::VideoCapture capture;
  capture.open(atoi(argv[2]));
  capture.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
  capture.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
  if (!capture.isOpened()) { cout << "error opening camera" << endl; return 1; }
  
  /* load calibration and setup system */
  cv::Mat frame;
  cv::Mat K, dist_coeff;
  cv::LocalizationSystem::load_matlab_calibration("../Calib_Results.m", K, dist_coeff);
  int number_of_circles = atoi(argv[1]);
  cv::LocalizationSystem system(number_of_circles, capture.get(CV_CAP_PROP_FRAME_WIDTH), capture.get(CV_CAP_PROP_FRAME_HEIGHT),
    K, dist_coeff, 0.123);
  
  /* setup gui and start capturing / processing */
  cvStartWindowThread();
  cv::namedWindow("output");
  cv::setMouseCallback("output", mouse_callback);
  bool axis_was_set = false;
  bool is_tracking = false;
  cv::Mat original_frame;
  int saved_frame_count = 0;
  while (true) {
    capture >> original_frame;
    original_frame.copyTo(frame);
    
    // set axis
    if (!axis_was_set && clicked) {
      cv::imwrite(output_directory + string("/axis.png"), original_frame);
      system.set_axis(original_frame); // set user-set axis 
      clicked = false;
      axis_was_set = true;
    }
    
    // draw axis
    if (axis_was_set) {
      string names[3] = { "center", "x", "y" };
      for (int i = 0; i < 3; i++) {
        cv::ellipse(frame, cv::Point(system.origin_circles[i].x, system.origin_circles[i].y), cv::Size((int)system.origin_circles[i].m0 * 2, (int)system.origin_circles[i].m1 * 2),
                          atan2(system.origin_circles[i].v1, system.origin_circles[i].v0), 0, 360, cv::Scalar(255,255,0), 2, CV_AA);
        cv::putText(frame, names[i], cv::Point(system.origin_circles[i].maxx, system.origin_circles[i].maxy), CV_FONT_HERSHEY_SIMPLEX,
                          1.0, cv::Scalar((i == 0) * 255, (i == 1) * 255, (i == 2) * 255), 3);
      }
      
      if (!is_tracking && clicked) {
        clicked = false;
        is_tracking = true;
        system.initialize(original_frame); // find circles in image
      }
    }
    
    // localize and draw circles
    if (is_tracking) {
      system.localize(original_frame); // track detected circles and localize
      for (int i = 0; i < number_of_circles; i++) {
        const cv::CircleDetector::Circle& circle = system.get_circle(i);
        cv::Vec2f coord = system.coordinates_transform * system.get_pose(circle).pos;
        cv::circle(frame, cv::Point(circle.x, circle.y), 3, cv::Scalar(255,255,0), -1);
        ostringstream ostr;
        ostr << fixed << setprecision(5) << coord;
        cv::putText(frame, ostr.str(), cv::Point(circle.maxx, circle.maxy), CV_FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255,0,255), 1.5, CV_AA);
      }
      
      // save frame
      if (clicked) {
        clicked = false;
        ostringstream ostr1;
        ostr1 << setw(4) << setfill('0') << saved_frame_count;
        string out_filename = output_directory + string("/frame") + ostr1.str();
        cv::imwrite(out_filename + string(".png"), original_frame);
        ofstream out_file((out_filename + string(".log")).c_str());
        out_file << "scale: " << unit_scale << endl;
        for (int i = 0; i < number_of_circles; i++) {
          cv::Vec2f coord = system.coordinates_transform * system.get_pose(0).pos * unit_scale;
          out_file << "circle " << i << " " << coord << endl;
        }
        saved_frame_count++;
      }
    }
    cv::imshow("output", frame);
  }
  return 0;
}

