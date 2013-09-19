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
#include <boost/program_options.hpp>
#include "localization_system.h"
using namespace std;
namespace po = boost::program_options;

bool stop = false;
void interrupt(int s) {
  stop = true;
}

bool clicked = false;
void mouse_callback(int event, int x, int y, int flags, void* param) {
  if (event == CV_EVENT_LBUTTONDOWN) clicked = true;
}

po::variables_map process_commandline(int argc, char** argv)
{
  po::options_description options_description("WhyCon options");
  options_description.add_options()
    ("help", "display this help")
    ("output,o", po::value<string>()->required(), "name to be used for all output files")
    ("circles,c", po::value<int>()->required(), "number of circles to be tracked (AFTER the calibration is performed)")
    ("cam", po::value<int>(), "use camera as input (expects id of camera)")
    ("video", po::value<string>(), "use video as input (expects path to video file)")
    ("img", po::value<string>(), "use sequence of images as input (expects pattern describing sequence)"
                                      "Use a pattern such as 'directory/%03d.png' for files named 000.png to "
                                      "999.png inside said directory")
    ("mat", po::value<string>(), "use matlab (.mat) calibration toolbox file for camera calibration parameters"
                                  " (expects path to file)")
    ("xml", po::value<string>(), "use 'calibrator' output file (.xml) for camera calibration parameters (expects path to file)")
  ;

  po::variables_map config_vars;
  po::store(po::parse_command_line(argc, argv, options_description), config_vars);
  if (config_vars.count("help")) { cerr << options_description << endl; exit(1); }

  try { po::notify(config_vars); }
  catch(po::error& e) { 
    cerr << endl << "ERROR: " << e.what() << endl << endl; 
    cerr << options_description << endl; 
    exit(1);
  } 

  try {
  if (!config_vars.count("mat") && !config_vars.count("xml"))
    throw std::runtime_error("Please specify one source for calibration parameters");
  if (!config_vars.count("cam") && !config_vars.count("video") && !config_vars.count("img"))
    throw std::runtime_error("Please specify one input source");
  if (config_vars["circles"].as<int>() < 0)
    throw std::runtime_error("Number of circles to track should be greater than 0");
  } catch(const std::runtime_error& e) {
    cerr << "ERROR: " << e.what() << endl;
    exit(1);
  }
  return config_vars;
}

int main(int argc, char** argv)
{
  signal(SIGINT, interrupt);

  /* process command line */
  po::variables_map config_vars = process_commandline(argc, argv);
  
  int number_of_circles = config_vars["circles"].as<int>();
  bool is_camera = config_vars.count("cam");
  std::string output_name(config_vars["output"].as<string>());
  
  /* setup camera */
  cv::VideoCapture capture;
  if (is_camera) {
    int cam_id = config_vars["cam"].as<int>();
    capture.open(cam_id);
    /*capture.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, 720);*/
  }
  else {
    std::string video_name(config_vars.count("img") ? config_vars["img"].as<string>() : config_vars["video"].as<string>());
    capture.open(video_name);    
  }
  if (!capture.isOpened()) { cout << "error opening camera/video" << endl; return 1; }
  
  /* load calibration and setup system */
  cv::Mat frame;
  cv::Mat K, dist_coeff;
  if (config_vars.count("xml"))
    cv::LocalizationSystem::load_opencv_calibration(config_vars["xml"].as<string>(), K, dist_coeff);
  else
    cv::LocalizationSystem::load_matlab_calibration(config_vars["mat"].as<string>(), K, dist_coeff);
    
  cv::LocalizationSystem system(number_of_circles, capture.get(CV_CAP_PROP_FRAME_WIDTH), capture.get(CV_CAP_PROP_FRAME_HEIGHT),
    K, dist_coeff);
    
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
      else cv::setTrackbarPos("frame", "output", saved_frame_idx);        
    }
    
    if (!capture.read(original_frame)) break;
    original_frame.copyTo(frame);
    
    // set axis
    if (!axis_was_set && clicked) {
      clicked = false;
      if (!system.set_axis(original_frame)) cout << "axis detection failed" << endl;
      else {
        axis_was_set = true;
        system.draw_axis(frame);
        writer << frame;
        ofstream data_file_axis((output_name + "_axis.log").c_str(), ios_base::out | ios_base::trunc);
        data_file_axis << "axis frame " << current_frame << endl;
        data_file_axis << "axis 0,0 " << system.get_pose(system.origin_circles[0]).pos << endl;
        data_file_axis << "axis 1,0 " << system.get_pose(system.origin_circles[1]).pos << endl;
        data_file_axis << "axis 0,1 " << system.get_pose(system.origin_circles[2]).pos << endl;
        data_file_axis << "axis 1,1 " << system.get_pose(system.origin_circles[3]).pos << endl;
        data_file_axis << "transform " << system.coordinates_transform << endl;
      }
    }
    
    // draw axis
    if (axis_was_set) {
      system.draw_axis(frame);
      
      if (!is_tracking && clicked) {
        clicked = false;
        is_tracking = true;
        cout << "initialization" << endl;
        system.initialize(original_frame); // find circles in image
        
      }
    }
    
    // localize and draw circles
    if (is_tracking) {
      cout << "tracking current frame" << endl;
      bool localized_correctly = system.localize(original_frame, (is_camera ? 1 : 50)); // track detected circles and localize
      
      if (localized_correctly) {
        for (int i = 0; i < number_of_circles; i++) {
          const cv::CircleDetector::Circle& circle = system.get_circle(i);
          cv::Vec3f coord = system.get_pose(circle).pos;
          cv::Vec3f coord_trans = system.get_transformed_pose(circle).pos;
          ostringstream ostr;
          //ostr << fixed << setprecision(2) << "[" << coord_trans(0) << "," << coord_trans(1) << "]";
          ostr << i;
          circle.draw(frame, ostr.str(), cv::Scalar(255,255,0));
          data_file << setprecision(15) << "frame " << saved_frame_idx + 1 << " circle " << i
            << " transformed: " << coord_trans(0) << " " << coord_trans(1) << " " << coord_trans(2)
            << " original: " << coord(0) << " " << coord(1) << " " << coord(2) << endl;
        }
      }
      
      writer << frame;
      saved_frame_idx++;
    }
    cv::imshow("output", frame);
  }
  return 0;
}

