#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>
using namespace std;

bool stop = false;
void interrupt(int s) {
  stop = true;
}

bool clicked = false, rclicked = false;
void mouse_callback(int event, int x, int y, int flags, void* param) {
  if (event == CV_EVENT_LBUTTONDOWN) clicked = true;
  if (event == CV_EVENT_RBUTTONDOWN) rclicked = true;
}

int main(int argc, char** argv) {
  if (argc < 3) {
    cout << "usage: calibrator <camera id> <squares in X> <squares in Y> <square X size> <square Y size>" << endl;
    return 1;
  }
  
  /* setup camera */
  cv::VideoCapture capture;
  capture.open(atoi(argv[1]));
  capture.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
  capture.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
  if (!capture.isOpened()) { cout << "error opening camera" << endl; return 1; }
  
  /* load calibration and setup system */
  cv::Mat frame;
  cv::Mat K, dist_coeff;
  
  /* setup gui and start capturing / processing */
  cvStartWindowThread();
  cv::namedWindow("output");
  cv::setMouseCallback("output", mouse_callback);
  
  int x_squares = atoi(argv[2]);
  int y_squares = atoi(argv[3]);
  cv::Size pattern_size(x_squares - 1, y_squares - 1);
  vector< vector<cv::Point2f> > all_corners;
  
  vector<cv::Point3f> grid3d;
  for(int i = 0; i < (x_squares - 1) * (y_squares - 1); i++)
    grid3d.push_back(cv::Point3f(i / (x_squares - 1), i % (x_squares - 1), 0.0f)); // TODO: set units here
  
  while (true) {
    capture >> frame;
    vector<cv::Point2f> corners;
    int result = cv::findChessboardCorners(frame, pattern_size, corners, CV_CALIB_CB_FILTER_QUADS | CV_CALIB_CB_FAST_CHECK);
    if (result) {
      cv::Mat gray;
      cv::cvtColor(frame, gray, CV_BGR2GRAY);
      cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 100, 0.05));
      cv::drawChessboardCorners(frame, pattern_size, cv::Mat(corners), result);
      
      if (clicked) {
        clicked = false;
        all_corners.push_back(corners);
      }
    }
    
    if (!all_corners.empty() && rclicked) {
      rclicked = false;
      vector< vector<cv::Point3f> > grid3d_all(all_corners.size(), grid3d);
      vector<cv::Mat> rotations, translations;
      double error = cv::calibrateCamera(grid3d_all, all_corners, frame.size(), K, dist_coeff, rotations, translations);
      cout << "K: " << K << endl;
      cout << "dist: " << dist_coeff << endl;
      cout << "reprojection error: " << error << endl;
      
      cv::FileStorage file("calibration.xml", cv::FileStorage::WRITE);
      file << "K" << K;
      file << "dist" << dist_coeff;
      return 0;
    }
    
    ostringstream ostr;
    ostr << "frames: " << all_corners.size();
    cv::putText(frame, ostr.str(), cv::Point(5, 15), CV_FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255,0,255), 1.5, CV_AA);
    cv::imshow("output", frame);
  }
  return 0;
}




