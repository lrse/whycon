#include <limits>
#include <fstream>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <gsl/gsl_math.h>
#include <gsl/gsl_eigen.h>
#include <sstream>
#include <iomanip>
#include <limits>
#include "localization_system.h"
#include "circle_detector.h"
using std::cout;
using std::endl;
using std::numeric_limits;

cv::LocalizationSystem::LocalizationSystem(int _targets, int _width, int _height, const cv::Mat& _K, const cv::Mat& _dist_coeff, 
  float _circle_diameter, float _diameter_ratio) :
  xscale(1), yscale(1), detector(_targets, _width, _height, _diameter_ratio), targets(_targets), width(_width), height(_height),  circle_diameter(_circle_diameter)
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
  return detector.initialize(image);
}

bool cv::LocalizationSystem::localize(const cv::Mat& image, int attempts) {
  for (int i = 0; i < attempts; i++) {
    cout << "localization attempt " << i << endl;
    if (detector.detect(image)) return true;
    else cout << "localization failed, not all circles detected" << i << endl;
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
  //cv::Vec3f result(-z0*z, -z1*z, z2*z); // NOTE: ZXY ordering XYZ
  cv::Vec3f result(z0*z, z1*z, z2*z); // NOTE: ZXY ordering XYZ
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
  return detector.circles[id];
}

cv::LocalizationSystem::Pose cv::LocalizationSystem::get_pose(int id)
{
  return get_pose(detector.circles[id]);
}

cv::LocalizationSystem::Pose cv::LocalizationSystem::get_transformed_pose(int id) {
  return get_transformed_pose(detector.circles[id]);
}

cv::LocalizationSystem::Pose cv::LocalizationSystem::get_transformed_pose(const cv::CircleDetector::Circle& circle)
{
  Pose pose;  
  pose.pos = coordinates_transform * get_pose(circle).pos;
  pose.pos(0) /= pose.pos(2);
  pose.pos(1) /= pose.pos(2);
  pose.pos(2) = 0;
  return pose;
}

// TODO: allow user to choose calibration circles, now the circles are read in the order of detection
bool cv::LocalizationSystem::set_axis(const cv::Mat& image)
{
  ManyCircleDetector axis_detector(4, width, height);
  if (!axis_detector.initialize(image)) return false;

  // get poses of each calibration circle
  /*float minx, miny;
  minx = miny = numeric_limits<float>::max();
  int zero_i;*/
  
  if (!axis_detector.detect(image)) return false;
  Pose circle_poses[4];
  for (int i = 0; i < 4; i++) {
    origin_circles[i] = axis_detector.circles[i];
    circle_poses[i] = get_pose(axis_detector.circles[i]);
    /*float x = circle_poses[i].pos(0);
    float y = circle_poses[i].pos(1);
    if (x < minx) { zero_i = i; x = minx; }
    if (y < miny) { zero_i = i; y = miny; }*/
  }

  // set (0,0) of circle at top, left
  /*std::swap(origin_circles[zero_i], origin_circles[0]);
  std::swap(circle_poses[zero_i], circle_poses[0]);*/
  cv::Vec3f vecs[3];  
  for (int i = 0; i < 3; i++) {
    vecs[i] = circle_poses[i + 1].pos - circle_poses[0].pos;
    cout << "vec " << i+1 << "->0 " << vecs[i] << endl;
  }
  int min_prod_i;
  float min_prod = numeric_limits<float>::max();
  for (int i = 0; i < 3; i++) {
    float prod = fabsf(vecs[(i + 2) % 3].dot(vecs[i]));
    cout << "prod: " << ((i + 2) % 3 + 1) << " " << i + 1 << " " << vecs[(i + 2) % 3] << " " << vecs[i] << " " << prod << endl;
    if (prod < min_prod) { min_prod = prod; min_prod_i = i; }
  }
  int axis1_i = (((min_prod_i + 2) % 3) + 1);
  int axis2_i = (min_prod_i + 1);
  if (fabsf(circle_poses[axis1_i].pos(0)) < fabsf(circle_poses[axis2_i].pos(0))) std::swap(axis1_i, axis2_i);
  int xy_i;
  for (int i = 1; i <= 3; i++) if (i != axis1_i && i != axis2_i) { xy_i = i; break; }
  cout << "axis ids: " << axis1_i << " " << axis2_i << " " << xy_i << endl;

  CircleDetector::Circle origin_circles_reordered[4];
  origin_circles_reordered[0] = origin_circles[0];
  origin_circles_reordered[1] = origin_circles[axis1_i];
  origin_circles_reordered[2] = origin_circles[axis2_i];
  origin_circles_reordered[3] = origin_circles[xy_i];
  for (int i = 0; i < 4; i++) {
    origin_circles[i] = origin_circles_reordered[i];
    circle_poses[i] = get_pose(origin_circles[i]);
    cout << "original poses: " << circle_poses[i].pos << endl;
  }
    
  float dim_x = 1.0;
  float dim_y = 1.0;
  cv::Vec2f targets[4] = { cv::Vec2f(0,0), cv::Vec2f(dim_x, 0), cv::Vec2f(0, dim_y), cv::Vec2f(dim_x, dim_y) };
  
  // build matrix of coefficients and independent term for linear eq. system
  cv::Mat A(8, 8, CV_64FC1), b(8, 1, CV_64FC1), x(8, 1, CV_64FC1);
  
  cv::Vec2f tmp[4];
  for (int i = 0; i < 4; i++) tmp[i] = cv::Vec2f(circle_poses[i].pos(0), circle_poses[i].pos(1)) / circle_poses[i].pos(2);
  for (int i = 0; i < 4; i++) {
    cv::Mat r_even = (cv::Mat_<double>(1, 8) << -tmp[i](0), -tmp[i](1), -1, 0, 0, 0, targets[i](0) * tmp[i](0), targets[i](0) * tmp[i](1));
    cv::Mat r_odd = (cv::Mat_<double>(1, 8) << 0, 0, 0, -tmp[i](0), -tmp[i](1), -1, targets[i](1) * tmp[i](0), targets[i](1) * tmp[i](1));
    r_even.copyTo(A.row(2 * i));
    r_odd.copyTo(A.row(2 * i + 1));    
    b.at<double>(2 * i)     = -targets[i](0);
    b.at<double>(2 * i + 1) = -targets[i](1);
  }

  // solve linear system and obtain transformation
  cv::solve(A, b, x);
  x.push_back(1.0);
  coordinates_transform = x.reshape(1, 3);

  // TODO: compare H obtained by OpenCV with the hand approach
  std::vector<cv::Vec2d> src(4), dsts(4);
  for (int i = 0; i < 4; i++) {
    src[i] = tmp[i];
    dsts[i] = targets[i];
    cout << tmp[i] << " -> " << targets[i] << endl;
  }
  cv::Matx33d H = cv::findHomography(src, dsts, CV_LMEDS);
  cout << "H " << H << endl;

  
  return true;
}

void cv::LocalizationSystem::draw_axis(cv::Mat& image)
{
  static string names[4] = { "0,0", "1,0", "0,1", "1,1" };
  for (int i = 0; i < 4; i++) {
    std::ostringstream ostr;
    //ostr << std::fixed << std::setprecision(5) << names[i] << endl << get_pose(origin_circles[i]).pos;
    origin_circles[i].draw(image, /*ostr.str()*/names[i], cv::Scalar((i == 0 || i == 3 ? 255 : 0), (i == 1 ? 255 : 0), (i == 2 || i == 3 ? 255 : 0)));
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
