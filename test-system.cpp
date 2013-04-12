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
  if (argc < 4) {
    cout << "usage: test-localizer <number of circles> [-cam <camera-number> | -video <video file>]  <output name>" << endl;
    return 1;
  }
  
  signal(SIGINT, interrupt);
  
  int number_of_circles = atoi(argv[1]);
  bool is_camera = (std::string(argv[2]) == "-cam");
  std::string output_name(argv[4]);
  
  /* setup camera */
  cv::VideoCapture capture;
  if (is_camera) {
    int cam_id = atoi(argv[3]);
    capture.open(cam_id);
    capture.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
  }
  else {
    std::string video_name(argv[3]);
    capture.open(video_name);    
  }
  if (!capture.isOpened()) { cout << "error opening camera/video" << endl; return 1; }
  
  /* load calibration and setup system */
  cv::Mat frame;
  cv::Mat K, dist_coeff;
  //cv::LocalizationSystem::load_opencv_calibration("calibration.xml.m", K, dist_coeff);
  cv::LocalizationSystem::load_matlab_calibration("../Calib_Results.m", K, dist_coeff);
  cv::LocalizationSystem system(number_of_circles, capture.get(CV_CAP_PROP_FRAME_WIDTH), capture.get(CV_CAP_PROP_FRAME_HEIGHT),
    K, dist_coeff, 0.123);
    
  /* create output directory */
  cv::Size frame_size(capture.get(CV_CAP_PROP_FRAME_WIDTH), capture.get(CV_CAP_PROP_FRAME_HEIGHT));
  cv::VideoWriter writer(output_name + ".avi", CV_FOURCC('M','J','P','G'), 15, frame_size);
  if (!writer.isOpened()) { cout << "error opening output video" << endl; return 1; }
  ofstream data_file((output_name + ".log").c_str(), ios_base::out | ios_base::trunc);
  if (!data_file) { cout << "error opening output data file" << endl; return 1; }
  
  /* setup gui and start capturing / processing */
  cvStartWindowThread();
  cv::namedWindow("output", CV_WINDOW_NORMAL);
  cv::setMouseCallback("output", mouse_callback);
  bool axis_was_set = false;
  bool is_tracking = false;
  cv::Mat original_frame;
  int saved_frame_idx = 0;
  
  int current_frame = 0;
  if (!is_camera) cv::createTrackbar("frame", "output", &current_frame, capture.get(CV_CAP_PROP_FRAME_COUNT) - 1);
  
  while (!stop) {
    if (!is_camera) {
      if (!axis_was_set || !is_tracking) {
        capture.set(CV_CAP_PROP_POS_FRAMES, current_frame);
        saved_frame_idx = current_frame;
      }
      else cv::setTrackbarPos("frame", "output", current_frame);        
    }
    
    if (!capture.read(original_frame)) break;
    original_frame.copyTo(frame);
    
    // set axis
    if (!axis_was_set && clicked) {
      system.set_axis(original_frame); // set user-set axis 
      clicked = false;
      axis_was_set = true;
      
      system.draw_axis(frame);
      writer << frame;
      ofstream data_file_axis((output_name + "_axis.log").c_str(), ios_base::out | ios_base::trunc);
      data_file_axis << "axis frame " << current_frame << endl;
      data_file_axis << "axis center " << system.get_pose(system.origin_circles[0]).pos << endl;
      data_file_axis << "axis x " << system.get_pose(system.origin_circles[1]).pos << endl;
      data_file_axis << "axis y " << system.get_pose(system.origin_circles[2]).pos << endl;
      data_file_axis << "transform " << system.coordinates_transform << endl;
    }
    
    // draw axis
    if (axis_was_set) {
      system.draw_axis(frame);
      
      if (!is_tracking && clicked) {
        clicked = false;
        is_tracking = true;
        system.initialize(original_frame); // find circles in image
        
      }
    }
    
    // localize and draw circles
    if (is_tracking) {
      bool localized_correctly = system.localize(original_frame, 50); // track detected circles and localize
      
      if (localized_correctly) {
        for (int i = 0; i < number_of_circles; i++) {
          const cv::CircleDetector::Circle& circle = system.get_circle(i);
          
          cv::Vec2f coord = system.coordinates_transform * system.get_pose(circle).pos;
          ostringstream ostr;
          ostr << fixed << setprecision(5) << coord;
          circle.draw(frame, ostr.str(), cv::Scalar(255,255,0));
        }
      }
      
      writer << frame;
      if (localized_correctly) {
        for (int i = 0; i < number_of_circles; i++) {
          cv::Vec3f coord3d = system.get_pose(i).pos;
          cv::Vec2f coord = system.coordinates_transform * system.get_pose(i).pos;
          data_file << "frame " << saved_frame_idx << " circle " << i
            << " 2d: " << coord(0) << " " << coord(1)
            << " 3d: " << coord3d(0) << " " << coord3d(1) << " " << coord3d(2) << endl;
        }
      }
      saved_frame_idx++;
    }
    cv::imshow("output", frame);
  }
  return 0;
}

