#include <limits>
#include <fstream>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include "localization_system.h"
#include "circle_detector.h"
using std::cout;
using std::endl;

cv::LocalizationSystem::LocalizationSystem(int _targets, int _width, int _height, const cv::Mat& K, const cv::Mat& dist_coeff, 
  float _circle_diamater) :
  targets(_targets), width(_width), height(_height), xscale(1), yscale(1), circle_diameter(_circle_diamater), localizer(_targets, _width, _height)
{
  fc[0] = K.at<double>(0,0);
  fc[1] = K.at<double>(1,1);
  cc[0] = K.at<double>(0,2);
  cc[1] = K.at<double>(1,2);
  
  cout.precision(30);
  cout << "fc " << fc[0] << " " << fc[1] << endl;
  cout << "cc " << cc[0] << " " << cc[1] << endl;
  cout << "kc ";
  for (int i = 0; i < 6; i++) {
    kc[i] = dist_coeff.at<double>(i);
    cout << kc[i] << " ";
  }
  kc[0] = 1; // WHY?
  cout << endl;  
}

bool cv::LocalizationSystem::initialize(const cv::Mat& image) {
  return localizer.initialize(image);
}

void cv::LocalizationSystem::localize(const cv::Mat& image) {
  localizer.localize(image);
}

cv::LocalizationSystem::Pose cv::LocalizationSystem::get_pose(const cv::CircleDetector::Circle& circle) {
  Pose result;
	float x,y,z,x1,x2,y1,y2,sx1,sx2,sy1,sy2;
	x = transform_x(circle.x,circle.y);
	y = transform_y(circle.x,circle.y);
	sx1 = circle.x + circle.v0 * circle.m0 * 2;
	sx2 = circle.x - circle.v0 * circle.m0 * 2;
	sy1 = circle.y + circle.v1 * circle.m0 * 2;
	sy2 = circle.y - circle.v1 * circle.m0 * 2;
	x1 = transform_x(sx1,sy1);
	x2 = transform_x(sx2,sy2);
	y1 = transform_y(sx1,sy1);
	y2 = transform_y(sx2,sy2);
	z = sqrtf((x1-x2) * (x1-x2) + (y1-y2) * (y1-y2));
	result.pos(0) = circle_diameter / z;
	result.pos(1) = -y * result.pos(0);
	result.pos(2) = -x * result.pos(0);
	result.rot(0) = acos(circle.m1 / circle.m0) / M_PI * 180.0;
	result.rot(1) = 0;
	result.rot(2) = 0;
  
  // TODO: scale?
	return result;
}

cv::LocalizationSystem::Pose cv::LocalizationSystem::get_pose(int id)
{
  return get_pose(localizer.circles[id]);
}

/* this assumes that the X axis is twice as long as the Y axis */
bool cv::LocalizationSystem::set_axis(const cv::Mat& image)
{
  CircleLocalizer axis_localizer(3, width, height);
  if (!axis_localizer.initialize(image)) return false;
  
  axis_localizer.localize(image);
  Pose circle_poses[3];
  for (int i = 0; i < 3; i++) {
    circle_poses[i] = get_pose(axis_localizer.circles[i]); 
  }
  
  vector<float> distances(3);
  /* note that indices of distances correspond to indices of first circle position */
  distances[0] = cv::norm(circle_poses[0].pos, circle_poses[1].pos);
  distances[1] = cv::norm(circle_poses[1].pos, circle_poses[2].pos);
  distances[2] = cv::norm(circle_poses[2].pos, circle_poses[0].pos);
  cout << "dist: " << distances[0] << " " << distances[1] << " " << distances[2] << endl;
  vector<float>::const_iterator max_it = std::max_element(distances.begin(), distances.end());
  vector<float>::const_iterator min_it = std::min_element(distances.begin(), distances.end());
  int min_idx = min_it - distances.begin();
  int max_idx = max_it - distances.begin();
  int middle_idx;
  for (int i = 0; i < 3; i++) { if (i != min_idx && i != max_idx) { middle_idx = i; break; } }
  
  /* determine which circle is which by looking which circle is shared by the minimum and second minimum length segment */
  int center_circle_idx, unit_one_circle_idx;
  if (min_idx == middle_idx || min_idx == (middle_idx + 1) % 3) {
    center_circle_idx = min_idx;
    unit_one_circle_idx = (min_idx + 1) % 3;
  }
  else {
    center_circle_idx = (min_idx + 1) % 3;
    unit_one_circle_idx = min_idx;
  }
  
  int unit_two_circle_idx;
  for (int i = 0; i < 3; i++) { if (i != center_circle_idx && i != unit_one_circle_idx) { unit_two_circle_idx = i; break; } }
  
  cv::Vec3f one = circle_poses[unit_one_circle_idx].pos;
  cv::Vec3f two = circle_poses[unit_two_circle_idx].pos;
  cv::Vec3f zero = circle_poses[center_circle_idx].pos;
  cv::Mat A = (Mat_<double>(6,6) <<
    one(0), one(1), one(2), 0, 0, 0,
    0, 0, 0, one(0), one(1), one(2),
    two(0), two(1), two(2), 0, 0, 0,
    0, 0, 0, two(0), two(1), two(2),
    zero(0), zero(1), zero(2), 0, 0, 0,
    0, 0, 0, zero(0), zero(1), zero(2)
  );
  cv::Mat x;
  cv::Mat b = (Mat_<double>(6,1) << 0, 1, 2, 0, 0, 0); // TODO: check order (X-Y or Y-X)?
  
  cout << "A " << A << endl;
  cout << "b " << b << endl;
  
  cv::solve(A, b, x, DECOMP_SVD);
  coordinates_transform = x.reshape(1, 2);
  
  origin_circles[0] = axis_localizer.circles[center_circle_idx];
  origin_circles[1] = axis_localizer.circles[unit_two_circle_idx];
  origin_circles[2] = axis_localizer.circles[unit_one_circle_idx];
  cout << "transform: " << coordinates_transform << endl;
  return true;
}

float cv::LocalizationSystem::unbarrel_x(float x, float y)
{
  x = (x-cc[0])/fc[0];
  y = (y-cc[1])/fc[1];
  float r = x*x+y*y;
  float dx = 2*kc[3]*x*y + kc[4]*(r + 2*x*x);
  float cx = (1-kc[1]*r-kc[2]*r*r-kc[5]*r*r*r)*x-dx;
  cx = (cx*fc[0]+cc[0]);
  return cx;
}

float cv::LocalizationSystem::unbarrel_y(float x, float y)
{
  x = (x-cc[0])/fc[0];
  y = (y-cc[1])/fc[1];
  float r = x*x+y*y;
  float dy = 2*kc[4]*x*y + kc[3]*(r + 2*y*y);
  float cy = (1-kc[1]*r-kc[2]*r*r-kc[5]*r*r*r)*y-dy;
  cy = (cy*fc[1]+cc[1]);
  return cy;
}

float cv::LocalizationSystem::transform_x(float xc,float yc)
{
	return (unbarrel_x(xc,yc)-cc[0])/fc[0];
}

float cv::LocalizationSystem::transform_y(float xc,float yc)
{
	return (unbarrel_y(xc,yc)-cc[1])/fc[1];
}

void cv::LocalizationSystem::load_matlab_calibration(const std::string& calib_file, cv::Mat& K, cv::Mat& dist_coeff)
{
  std::ifstream file(calib_file.c_str());
  if (!file) throw std::runtime_error("calibration file not found");
  std::string line;
  
  K.create(3, 3, CV_64FC1);
  dist_coeff.create(6, 1, CV_64FC1);
  dist_coeff = cv::Scalar(0);
  
  while (std::getline(file, line)) {
    std::string s;
    std::istringstream istr(line);
    istr >> s;
    if (s == "fc") {
      istr >> s >> s;
      istr >> K.at<double>(0,0) >> s;
      istr >> K.at<double>(1,1);
    }
    else if (s == "cc") {
      cout << "cosa: " << K.at<double>(1,1) << endl;
      istr >> s >> s >> K.at<double>(0,2);
      istr >> s >> K.at<double>(1,2);
    }
    else if (s == "kc") {
      istr >> s >> s;
      int i = 0; 
      do {
        istr >> dist_coeff.at<double>(i) >> s;
        i++;
      } while (s != "];");
    }
  }    
}
