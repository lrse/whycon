#include <limits>
#include <fstream>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <gsl/gsl_math.h>
#include <gsl/gsl_eigen.h>
#include "localization_system.h"
#include "circle_detector.h"
using std::cout;
using std::endl;

cv::LocalizationSystem::LocalizationSystem(int _targets, int _width, int _height, const cv::Mat& _K, const cv::Mat& _dist_coeff, 
  float _circle_diameter) :
  xscale(1), yscale(1), localizer(_targets, _width, _height), targets(_targets), width(_width), height(_height),  circle_diameter(_circle_diameter)
{
  _K.copyTo(K);
  _dist_coeff.copyTo(dist_coeff);
  
  fc[0] = K.at<double>(0,0);
  fc[1] = K.at<double>(1,1);
  cc[0] = K.at<double>(0,2);
  cc[1] = K.at<double>(1,2);
  
  cout.precision(30);
  cout << "fc " << fc[0] << " " << fc[1] << endl;
  cout << "cc " << cc[0] << " " << cc[1] << endl;
  kc[0] = 1;
  cout << "kc " << kc[0] << " ";
  for (int i = 0; i < 5; i++) {
    kc[i + 1] = dist_coeff.at<double>(i);
    cout << kc[i + 1] << " ";
  }
  cout << endl;  
}

bool cv::LocalizationSystem::initialize(const cv::Mat& image) {
  return localizer.initialize(image);
}

bool cv::LocalizationSystem::localize(const cv::Mat& image, int attempts) {
  for (int i = 0; i < attempts; i++) {
    if (localizer.localize(image)) return true;
    else cout << "localization failed, attempt " << i << endl;
  }
  return false;
}

     
/* TODO: use cv::eigen */
cv::Vec3f cv::LocalizationSystem::eigen(double data[])
{
	gsl_matrix_view m = gsl_matrix_view_array (data, 3, 3);
	gsl_vector *eval = gsl_vector_alloc (3);
	gsl_matrix *evec = gsl_matrix_alloc (3, 3);

	gsl_eigen_symmv_workspace * w = gsl_eigen_symmv_alloc (3);
	gsl_eigen_symmv (&m.matrix, eval, evec, w);
	gsl_eigen_symmv_free (w);
	gsl_eigen_symmv_sort (eval, evec,GSL_EIGEN_SORT_ABS_ASC);

	float L1 =gsl_vector_get(eval,1);
	float L2 =gsl_vector_get(eval,2);
	float L3 =gsl_vector_get(eval,0);
	int V2=2;
	int V3=0;

	float z = circle_diameter/sqrt(-L2*L3)/2.0;
	float z0 = +L3*sqrt((L2-L1)/(L2-L3))*gsl_matrix_get(evec,0,V2)+L2*sqrt((L1-L3)/(L2-L3))*gsl_matrix_get(evec,0,V3);
	float z1 = +L3*sqrt((L2-L1)/(L2-L3))*gsl_matrix_get(evec,1,V2)+L2*sqrt((L1-L3)/(L2-L3))*gsl_matrix_get(evec,1,V3);
	float z2 = +L3*sqrt((L2-L1)/(L2-L3))*gsl_matrix_get(evec,2,V2)+L2*sqrt((L1-L3)/(L2-L3))*gsl_matrix_get(evec,2,V3);
	if (z2*z < 0){
		 z2 = -z2;
		 z1 = -z1;
		 z0 = -z0;
	}
  cv::Vec3f result(z2*z, -z0*z, -z1*z);
	gsl_vector_free (eval);
	gsl_matrix_free (evec);

	return result;
}


cv::LocalizationSystem::Pose cv::LocalizationSystem::get_pose(const cv::CircleDetector::Circle& circle) {
  Pose result;
	float x,y,x1,x2,y1,y2,sx1,sx2,sy1,sy2,major,minor,v0,v1;
  
  //transform the center
	x = transform_x(circle.x,circle.y);
	y = transform_y(circle.x,circle.y);
  
  //calculate the major axis 
	//endpoints in image coords
	sx1 = circle.x + circle.v0 * circle.m0 * 2;
	sx2 = circle.x - circle.v0 * circle.m0 * 2;
	sy1 = circle.y + circle.v1 * circle.m0 * 2;
	sy2 = circle.y - circle.v1 * circle.m0 * 2;
  //endpoints in camera coords 
	x1 = transform_x(sx1,sy1);
	x2 = transform_x(sx2,sy2);
	y1 = transform_y(sx1,sy1);
	y2 = transform_y(sx2,sy2);
  //semiaxis length 
	major = sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))/2.0;
	v0 = (x2-x1)/major/2.0;
	v1 = (y2-y1)/major/2.0;

	//calculate the minor axis 
	//endpoints in image coords
	sx1 = circle.x + circle.v1 * circle.m1 * 2;
	sx2 = circle.x - circle.v1 * circle.m1 * 2;
	sy1 = circle.y - circle.v0 * circle.m1 * 2;
	sy2 = circle.y + circle.v0 * circle.m1 * 2;
	//endpoints in camera coords 
	x1 = transform_x(sx1,sy1);
	x2 = transform_x(sx2,sy2);
	y1 = transform_y(sx1,sy1);
	y2 = transform_y(sx2,sy2);
	//semiaxis length 
	minor = sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))/2.0;

	//construct the conic
	float a,b,c,d,e,f;
	a = v0*v0/(major*major)+v1*v1/(minor*minor);
	b = v0*v1*(1/(major*major)-1/(minor*minor));
	c = v0*v0/(minor*minor)+v1*v1/(major*major);
	d = (-x*a-b*y);
	e = (-y*c-b*x);
	f = (a*x*x+c*y*y+2*b*x*y-1);
	double data[] ={a,b,d,b,c,e,d,e,f}; 

  result.pos = eigen(data);
  result.rot(0) = acos(circle.m1/circle.m0)/M_PI*180.0;
	result.rot(1) = atan2(circle.v1,circle.v0)/M_PI*180.0;
	result.rot(2) = circle.v1/circle.v0;
  
  return result;
}

const cv::CircleDetector::Circle& cv::LocalizationSystem::get_circle(int id)
{
  return localizer.circles[id];
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
  
  cout << "poses: " << circle_poses[0].pos << " " << circle_poses[1].pos << " " << circle_poses[2].pos << endl;
  vector<float>::const_iterator max_it = std::max_element(distances.begin(), distances.end());
  vector<float>::const_iterator min_it = std::min_element(distances.begin(), distances.end());
  int min_idx = min_it - distances.begin();
  int max_idx = max_it - distances.begin();
  int middle_idx = -1;
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
  
  int unit_two_circle_idx = -1;
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
  float longer_axis_scale = 2;
  float shorter_axis_scale = 1;
  cv::Mat b = (Mat_<double>(6,1) << 0, shorter_axis_scale, longer_axis_scale, 0, 0, 0); // TODO: check order (X-Y or Y-X)?
  
  cout << "A " << A << endl;
  cout << "b " << b << endl;
  
  cv::solve(A, b, x, DECOMP_SVD);
  coordinates_transform = x.reshape(1, 2);
  
  origin_circles[0] = axis_localizer.circles[center_circle_idx];
  origin_circles[1] = axis_localizer.circles[unit_two_circle_idx];
  origin_circles[2] = axis_localizer.circles[unit_one_circle_idx];
  
  vector<float> ortogonality(3);
  /*ortogonality[0] = (origin_circles[1].pos - origin_circles[0].pos).dot(origin_circles[2].pos - origin_circles[1].pos);
  ortogonality[1] = (origin_circles[2].pos - origin_circles[1].pos).dot(origin_circles[2].pos - origin_circles[0].pos);
  ortogonality[2] = (origin_circles[2].pos - origin_circles[0].pos).dot(origin_circles[1].pos - origin_circles[0].pos);*/
  ortogonality[0] = (one - zero).dot(two - zero);
  ortogonality[1] = (two - zero).dot(two - one);
  ortogonality[2] = (one - two).dot(zero - two);
  cout << "ort: " << ortogonality[0] << " " << ortogonality[1] << " " << ortogonality[2] << endl;
  cout << "segm: " << axis_localizer.circles[0].x << " " << axis_localizer.circles[0].y << endl;
  cout << "segm: " << axis_localizer.circles[1].x << " " << axis_localizer.circles[1].y << endl;
  cout << "segm: " << axis_localizer.circles[2].x << " " << axis_localizer.circles[2].y << endl;
  
  cout << "transform: " << coordinates_transform << endl;
  return true;
}

void cv::LocalizationSystem::draw_axis(cv::Mat& image)
{
  static string names[3] = { "center", "x", "y" };
  for (int i = 0; i < 3; i++) {
    origin_circles[i].draw(image, names[i], cv::Scalar((i == 0 ? 255 : 0), (i == 1 ? 255 : 0), (i == 2 ? 255 : 0)));
  }
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
  
  K = cv::Mat::eye(3, 3, CV_64FC1);
  dist_coeff.create(5, 1, CV_64FC1);
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

void cv::LocalizationSystem::load_opencv_calibration(const std::string& calib_file, cv::Mat& K, cv::Mat& dist_coeff) {
  cv::FileStorage file(calib_file, cv::FileStorage::READ);
  if (!file.isOpened()) throw std::runtime_error("calibration file not found");
  
  file["K"] >> K;
  file["dist"] >> dist_coeff;
}
