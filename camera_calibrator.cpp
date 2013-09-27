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
  if (argc != 8 && argc != 9) {
    cout << "usage: calibrator <width> <height> <squares in X> <squares in Y> <square X size [mm]> <square Y size [mm]> <camera id> [<camera id>]" << endl;
    cout << "X,Y direction is width,height in image" << endl;
    return 1;
  }
  
  /* setup camera */
  int width = atoi(argv[1]);
  int height = atoi(argv[2]);

  cv::VideoCapture capture1, capture2;
  capture1.open(atoi(argv[7]));
  capture1.set(CV_CAP_PROP_FRAME_WIDTH, width);
  capture1.set(CV_CAP_PROP_FRAME_HEIGHT, height);
  capture1.set(CV_CAP_PROP_FPS, 20);
  if (!capture1.isOpened()) { cout << "error opening first camera" << endl; return 1; }
  
    
  bool do_stereo = false;
  if (argc == 9) {
    capture2.open(atoi(argv[8]));
    capture2.set(CV_CAP_PROP_FRAME_WIDTH, width);
    capture2.set(CV_CAP_PROP_FRAME_HEIGHT, height);
    capture2.set(CV_CAP_PROP_FPS, 20);
    if (!capture2.isOpened()) { cout << "error opening second camera" << endl; return 1; }
    do_stereo = true;
  }
  
  /* load calibration and setup system */
  cv::Mat frame1, frame2;
  cv::Mat K1, K2, dist_coeff1, dist_coeff2;
  
  /* setup gui and start capturing / processing */
  cvStartWindowThread();
  cv::namedWindow("first");
  if (do_stereo) cv::namedWindow("second");
  cv::setMouseCallback("first", mouse_callback);
  
  int x_squares = atoi(argv[3]);
  int y_squares = atoi(argv[4]);
  float x_size = atof(argv[5]);
  float y_size = atof(argv[6]);
  cv::Size pattern_size(x_squares - 1, y_squares - 1);
  vector< vector<cv::Point2f> > all_corners;
  
  vector<cv::Point3f> grid3d;
  for(int i = 0; i < (x_squares - 1) * (y_squares - 1); i++)
    grid3d.push_back(cv::Point3f((i / (x_squares - 1)) * x_size, (i % (x_squares - 1)) * y_size, 0.0f)); // TODO: set units here
  
  while (true) {    
    capture1.grab();
    capture1.retrieve(frame1);
    if (do_stereo) capture2.grab();
    
    if (do_stereo) capture2.retrieve(frame2);

    vector<cv::Point2f> corners;
    int result = cv::findChessboardCorners(frame1, pattern_size, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
    if (result) {
      cv::Mat gray;
      cv::cvtColor(frame1, gray, CV_BGR2GRAY);
      cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 100, 0.05));
      cv::drawChessboardCorners(frame1, pattern_size, cv::Mat(corners), result);
      
      if (clicked) {
        clicked = false;
        all_corners.push_back(corners);
      }
    }
    
    if (!all_corners.empty() && rclicked) {
      rclicked = false;
      vector< vector<cv::Point3f> > grid3d_all(all_corners.size(), grid3d);
      vector<cv::Mat> rotations, translations;
      int flags = 0;
      for (int i = 0; i < 5; i++) {
        cout << "iteration " << i << endl;
        double error = cv::calibrateCamera(grid3d_all, all_corners, frame1.size(), K1, dist_coeff1, rotations, translations, flags);
        cout << "K: " << K1 << endl;
        cout << "dist: " << dist_coeff1 << endl;
        cout << "reprojection error: " << error << endl;
        flags = CV_CALIB_USE_INTRINSIC_GUESS;
      }
      
      cv::FileStorage file("calibration.xml", cv::FileStorage::WRITE);
      file << "K" << K1;
      file << "dist" << dist_coeff1;
      return 0;
    }
    
    ostringstream ostr;
    ostr << "frames: " << all_corners.size();
    cv::putText(frame1, ostr.str(), cv::Point(5, 15), CV_FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255,0,255), 1.5, CV_AA);
    
    if (!frame1.empty()) cv::imshow("first", frame1);
    if (do_stereo && !frame2.empty()) cv::imshow("second", frame2);
  }
  return 0;
}




