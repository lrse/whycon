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
#include "localization_viewer.h"
using namespace std;
namespace po = boost::program_options;

bool stop = false;
void interrupt(int s) {
  stop = true;
}

bool clicked = false;
void mouse_callback(int event, int x, int y, int flags, void* param) {
  if (event == CV_EVENT_LBUTTONDOWN) { cout << "clicked window" << endl; clicked = true; }
}

bool do_tracking;
bool use_gui;
int number_of_targets;

po::variables_map process_commandline(int argc, char** argv)
{
  po::options_description options_description("WhyCon options");
  options_description.add_options()
    ("help", "display this help")
    
    ("set-axis", po::value<string>(), "perform axis detection and save results to specified XML file")
    ("track", po::value<int>(), "perform tracking of the specified ammount of targets")
    
    ("cam", po::value<int>(), "use camera as input (expects id of camera)")
    ("video", po::value<string>(), "use video as input (expects path to video file)")
    ("img", po::value<string>(), "use sequence of images as input (expects pattern describing sequence)"
                                      "Use a pattern such as 'directory/%03d.png' for files named 000.png to "
                                      "999.png inside said directory")

    ("output,o", po::value<string>(), "name to be used for all tracking output files")
    
    ("axis", po::value<string>(), "use specified XML file for axis definition during tracking")
    
    ("mat", po::value<string>(), "use specified matlab (.m) calibration toolbox file for camera calibration parameters")
    ("xml", po::value<string>(), "use specified 'camera_calibrator' file (.xml) for camera calibration parameters")
    
    ("no-gui,n", "disable opening of GUI")
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

    use_gui = !config_vars.count("no-gui");

    if (config_vars.count("track")) do_tracking = true;
    else if (config_vars.count("set-axis")) do_tracking = false;
    else throw std::runtime_error("Select either tracking or axis setting mode");

    if (do_tracking) {
      if (!config_vars.count("output")) throw std::runtime_error("Specify output name of files");
      if (config_vars["track"].as<int>() < 0) throw std::runtime_error("Number of circles to track should be greater than 0");
      if (!config_vars.count("axis")) throw std::runtime_error("Axis definition file is required for tracking");
      
      number_of_targets = config_vars["track"].as<int>();
    }
    else {
      if (config_vars.count("video")) throw std::runtime_error("Video input is not supported for axis definition");
      if (!use_gui && config_vars.count("cam")) throw std::runtime_error("Camera input is not supported for axis setting when GUI is disabled");
    }
  }
  catch(const std::runtime_error& e) {
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

  /* setup input */
  bool is_camera = config_vars.count("cam");
  cv::VideoCapture capture;
  if (is_camera) {
    int cam_id = config_vars["cam"].as<int>();
    capture.open(cam_id);
  }
  else {
    std::string video_name(config_vars.count("img") ? config_vars["img"].as<string>() : config_vars["video"].as<string>());
    capture.open(video_name);    
  }
  if (!capture.isOpened()) { cout << "error opening camera/video" << endl; return 1; }

  /* load calibration */
  cv::Mat K, dist_coeff;
  if (config_vars.count("xml"))
    cv::LocalizationSystem::load_opencv_calibration(config_vars["xml"].as<string>(), K, dist_coeff);
  else
    cv::LocalizationSystem::load_matlab_calibration(config_vars["mat"].as<string>(), K, dist_coeff);

  /* init system */
  cv::Size frame_size(capture.get(CV_CAP_PROP_FRAME_WIDTH), capture.get(CV_CAP_PROP_FRAME_HEIGHT));
  cv::LocalizationSystem system(number_of_targets, frame_size.width, frame_size.height, K, dist_coeff);

  #ifdef ENABLE_VIEWER
  cv::LocalizationViewer viewer(system);
  if (use_gui) viewer.start();
  #endif

  if (use_gui) {
    cvStartWindowThread();
    cv::namedWindow("output", CV_WINDOW_NORMAL);
    cv::setMouseCallback("output", mouse_callback);
  }

  //std::string output_name(config_vars["output"].as<string>());
    
  /* create output directory */
  
  /*cv::VideoWriter writer(output_name + ".avi", CV_FOURCC('M','J','P','G'), 15, frame_size);
  if (!writer.isOpened()) { cout << "error opening output video" << endl; return 1; }
  ofstream data_file((output_name + ".log").c_str(), ios_base::out | ios_base::trunc);
  if (!data_file) { cout << "error opening output data file" << endl; return 1; }*/
  
  /* setup gui and start capturing / processing */
  //int current_frame = 0, last_frame = 0;
  //bool axis_was_set = false;
  bool is_tracking = false;
  if (!is_camera) clicked = true; // when not using camera, emulate user click so that tracking starts immediately
  cv::Mat original_frame, frame;
  int saved_frame_idx = 0;
  
  /*int total_frames;
  if (!is_camera) total_frames = capture.get(CV_CAP_PROP_FRAME_COUNT);*/

  if (do_tracking) {
    system.read_axis(config_vars["axis"].as<string>());
  }

  while (!stop) {
    /*if (!is_camera && use_gui) {
      if (!axis_was_set || !is_tracking) {
        capture.set(CV_CAP_PROP_POS_FRAMES, current_frame);
        saved_frame_idx = current_frame;
      }
      else if (total_frames > 1) cv::setTrackbarPos("frame", "output", saved_frame_idx);        
    }*/

    if (!capture.read(original_frame)) { cout << "no more frames left to read" << endl; break; }
    original_frame.copyTo(frame);

    if (!do_tracking) {
      if (!is_camera || clicked) {
        bool axis_was_set = system.set_axis(original_frame, config_vars["set-axis"].as<string>());
        if (!axis_was_set) throw std::runtime_error("Error setting axis!");      
        system.draw_axis(frame);
        cv::imwrite("axis_detected.png", frame);
        /*writer << output_frame;
        ofstream data_file_axis((output_name + "_axis.log").c_str(), ios_base::out | ios_base::trunc);
        data_file_axis << "axis 0,0 " << system.get_pose(system.origin_circles[0]).pos << endl;
        data_file_axis << "axis 1,0 " << system.get_pose(system.origin_circles[1]).pos << endl;
        data_file_axis << "axis 0,1 " << system.get_pose(system.origin_circles[2]).pos << endl;
        data_file_axis << "axis 1,1 " << system.get_pose(system.origin_circles[3]).pos << endl;
        data_file_axis << "transform " << system.coordinates_transform << endl;*/
        stop = true;
      }
      if (use_gui) cv::imshow("output", frame);
    }
    else {
      if (!is_tracking && (!use_gui || clicked)) {
        clicked = false;
        is_tracking = true;
        cout << "initialization" << endl;
        system.initialize(original_frame); // find circles in image      
      }
      
      // localize and draw circles
      if (is_tracking) {
        cout << "tracking current frame" << endl;
        bool localized_correctly = system.localize(original_frame, (is_camera ? 1 : 50)); // track detected circles and localize
        
        if (localized_correctly) {
          for (int i = 0; i < number_of_targets; i++) {
            const cv::CircleDetector::Circle& circle = system.get_circle(i);
            cv::Vec3f coord = system.get_pose(circle).pos;
            cv::Vec3f coord_trans = system.get_transformed_pose(circle).pos;
            ostringstream ostr;
            //ostr << fixed << setprecision(2) << "[" << coord_trans(0) << "," << coord_trans(1) << "]";
            ostr << i;
            if (use_gui) circle.draw(frame, ostr.str(), cv::Scalar(255,255,0));
            /*data_file << setprecision(15) << "frame " << saved_frame_idx + 1 << " circle " << i
              << " transformed: " << coord_trans(0) << " " << coord_trans(1) << " " << coord_trans(2)
              << " original: " << coord(0) << " " << coord(1) << " " << coord(2) << endl;*/
          }
          #ifdef ENABLE_VIEWER
          if (use_gui) viewer.update();
          #endif      
        }

        //writer << frame;
      }
      if (use_gui) cv::imshow("output", frame);
    }
  }

  /*#ifdef ENABLE_VIEWER
  if (!stop) viewer.wait();
  #endif*/
  return 0;
}

