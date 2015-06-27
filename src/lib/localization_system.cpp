#include <limits>
#include <fstream>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <sstream>
#include <iomanip>
#include <limits>
#include <whycon/circle_detector.h>
#include <whycon/localization_system.h>
using std::cout;
using std::endl;
using std::numeric_limits;

cv::LocalizationSystem::LocalizationSystem(int _targets, int _width, int _height, const cv::Mat& _K, const cv::Mat& _dist_coeff, 
  float _outer_diameter, float _inner_diameter) :
  xscale(1), yscale(1), detector(_targets, _width, _height, _inner_diameter / _outer_diameter),
  targets(_targets), width(_width), height(_height), axis_set(false), circle_diameter(_outer_diameter)
{
  _K.copyTo(K);
  _dist_coeff.copyTo(dist_coeff);
  
  fc[0] = K.at<double>(0,0);
  fc[1] = K.at<double>(1,1);
  cc[0] = K.at<double>(0,2);
  cc[1] = K.at<double>(1,2);
  
  kc[0] = 1;
  for (int i = 0; i < 5; i++) kc[i + 1] = dist_coeff.at<double>(i);

  coordinates_transform = cv::Matx33f(1, 0, 0, 0, 1, 0, 0, 0, 1);
  precompute_undistort_map();

  cout.precision(30);
}

bool cv::LocalizationSystem::localize(const cv::Mat& image, bool reset, int attempts, int max_refine) {
  return detector.detect(image, reset, attempts, max_refine);
}

cv::LocalizationSystem::Pose cv::LocalizationSystem::get_pose(const cv::CircleDetector::Circle& circle) const {
  Pose result;
  double x,y,x1,x2,y1,y2,sx1,sx2,sy1,sy2,major,minor,v0,v1;
  
  //transform the center
	transform(circle.x,circle.y, x, y);
  
  //calculate the major axis 
	//endpoints in image coords
	sx1 = circle.x + circle.v0 * circle.m0 * 2;
	sx2 = circle.x - circle.v0 * circle.m0 * 2;
	sy1 = circle.y + circle.v1 * circle.m0 * 2;
	sy2 = circle.y - circle.v1 * circle.m0 * 2;

  //endpoints in camera coords 
	transform(sx1, sy1, x1, y1);
	transform(sx2, sy2, x2, y2);

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
	transform(sx1, sy1, x1, y1);
	transform(sx2, sy2, x2, y2);

	//semiaxis length 
	minor = sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))/2.0;

	//construct the conic
	double a,b,c,d,e,f;
	a = v0*v0/(major*major)+v1*v1/(minor*minor);
	b = v0*v1*(1/(major*major)-1/(minor*minor));
	c = v0*v0/(minor*minor)+v1*v1/(major*major);
	d = (-x*a-b*y);
	e = (-y*c-b*x);
	f = (a*x*x+c*y*y+2*b*x*y-1);
	cv::Matx33d data(a,b,d,
									 b,c,e,
									 d,e,f);

	// compute conic eigenvalues and eigenvectors
	cv::Vec3d eigenvalues;
	cv::Matx33d eigenvectors;
	cv::eigen(data, eigenvalues, eigenvectors);

	// compute ellipse parameters in real-world
	double L1 = eigenvalues(1);
	double L2 = eigenvalues(0);
	double L3 = eigenvalues(2);
	int V2 = 0;
	int V3 = 2;

	// position
	double z = circle_diameter/sqrt(-L2*L3)/2.0;
	cv::Matx13d position_mat = L3 * sqrt((L2 - L1) / (L2 - L3)) * eigenvectors.row(V2) + L2 * sqrt((L1 - L3) / (L2 - L3)) * eigenvectors.row(V3);
	result.pos = cv::Vec3f(position_mat(0), position_mat(1), position_mat(2));
	int S3 = (result.pos(2) * z < 0 ? -1 : 1);
	result.pos *= S3 * z;

	WHYCON_DEBUG("ellipse center: " << x << "," << y << " " << " computed position: " << result.pos << " " << result.pos / result.pos(2));

	// rotation
	cv::Matx13d normal_mat = sqrt((L2 - L1) / (L2 - L3)) * eigenvectors.row(V2) + sqrt((L1 - L3) / (L2 - L3)) * eigenvectors.row(V3);
	cv::normalize(cv::Vec3f(normal_mat(0), normal_mat(1), normal_mat(2)), result.rot, 1, cv::NORM_L2SQR);
	result.rot(0) = atan2(result.rot(1), result.rot(0));
	result.rot(1) = acos(result.rot(2));
	result.rot(2) = 0; /* not recoverable */
	/* TODO: to be checked */

  return result;
}

const cv::CircleDetector::Circle& cv::LocalizationSystem::get_circle(int id)
{
  return detector.circles[id];
}

cv::LocalizationSystem::Pose cv::LocalizationSystem::get_pose(int id) const
{
  return get_pose(detector.circles[id]);
}

cv::LocalizationSystem::Pose cv::LocalizationSystem::get_transformed_pose(int id) const {
  return get_transformed_pose(detector.circles[id]);
}

cv::LocalizationSystem::Pose cv::LocalizationSystem::get_transformed_pose(const cv::CircleDetector::Circle& circle) const
{
  Pose pose;  
  pose.pos = coordinates_transform * get_pose(circle).pos;
  pose.pos(0) /= pose.pos(2) * xscale;
  pose.pos(1) /= pose.pos(2) * yscale;
  pose.pos(2) = 0;
  return pose;
}

// TODO: allow user to choose calibration circles, now the circles are read in the order of detection
bool cv::LocalizationSystem::set_axis(const cv::Mat& image, int max_attempts, int refine_steps, const std::string& file)
{
  ManyCircleDetector axis_detector(4, width, height);
  if (!axis_detector.detect(image, true, max_attempts, refine_steps)) return false;

  Pose circle_poses[4];
  for (int i = 0; i < 4; i++) {
    origin_circles[i] = axis_detector.circles[i];
    circle_poses[i] = get_pose(axis_detector.circles[i]);
  }

  cv::Vec3f vecs[3];  
  for (int i = 0; i < 3; i++) {
    vecs[i] = circle_poses[i + 1].pos - circle_poses[0].pos;
    cout << "vec " << i+1 << "->0 " << vecs[i] << endl;
  }
  int min_prod_i = 0;
  float min_prod = numeric_limits<float>::max();
  for (int i = 0; i < 3; i++) {
    float prod = fabsf(vecs[(i + 2) % 3].dot(vecs[i]));
    cout << "prod: " << ((i + 2) % 3 + 1) << " " << i + 1 << " " << vecs[(i + 2) % 3] << " " << vecs[i] << " " << prod << endl;
    if (prod < min_prod) { min_prod = prod; min_prod_i = i; }
  }
  int axis1_i = (((min_prod_i + 2) % 3) + 1);
  int axis2_i = (min_prod_i + 1);
  if (fabsf(circle_poses[axis1_i].pos(0)) < fabsf(circle_poses[axis2_i].pos(0))) std::swap(axis1_i, axis2_i);
  int xy_i = 0;
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
  cout << "H " << coordinates_transform << endl;

  // TODO: compare H obtained by OpenCV with the hand approach
  std::vector<cv::Vec2f> src(4), dsts(4);
  for (int i = 0; i < 4; i++) {
    src[i] = tmp[i];
    dsts[i] = targets[i];
    cout << tmp[i] << " -> " << targets[i] << endl;
  }

  /*cv::Matx33f H = cv::findHomography(src, dsts, CV_LMEDS);
  cout << "OpenCV H " << H << endl;*/

  if (!file.empty()) {
    cv::FileStorage fs(file, cv::FileStorage::WRITE);
    fs << "H" << cv::Mat(cv::Matx33d(coordinates_transform)); // store as double to get more decimals
    fs << "c0"; origin_circles[0].write(fs);
    fs << "c1"; origin_circles[1].write(fs);
    fs << "c2"; origin_circles[2].write(fs);
    fs << "c3"; origin_circles[3].write(fs);
  }
  axis_set = true;
  return true;
}

void cv::LocalizationSystem::read_axis(const std::string& file, float _xscale, float _yscale) {
  cv::FileStorage fs(file, cv::FileStorage::READ);
  if (!fs.isOpened()) throw std::runtime_error("could not axis open file");
  cv::Mat m;
  fs["H"] >> m;
  coordinates_transform = cv::Matx33f(m);
  origin_circles[0].read(fs["c0"]);
  origin_circles[1].read(fs["c1"]);
  origin_circles[2].read(fs["c2"]);
  origin_circles[3].read(fs["c3"]);

  xscale = _xscale;
  yscale = _yscale;

  axis_set = true;
  WHYCON_DEBUG("transformation: " << coordinates_transform);
}

void cv::LocalizationSystem::draw_axis(cv::Mat& image)
{
  static std::string names[4] = { "0,0", "1,0", "0,1", "1,1" };
  for (int i = 0; i < 4; i++) {
    std::ostringstream ostr;
    //ostr << std::fixed << std::setprecision(5) << names[i] << endl << get_pose(origin_circles[i]).pos;
    origin_circles[i].draw(image, /*ostr.str()*/names[i], cv::Vec3b((i == 0 || i == 3 ? 255 : 0), (i == 1 ? 255 : 0), (i == 2 || i == 3 ? 255 : 0)));
  }
}

/* normalize coordinates: move from image to canonical and remove distortion */
void cv::LocalizationSystem::transform(double x_in, double y_in, double& x_out, double& y_out) const
{
  #if defined(ENABLE_FULL_UNDISTORT)
  x_out = (x_in-cc[0])/fc[0];
  y_out = (y_in-cc[1])/fc[1];
  #else
  std::vector<cv::Vec2d> src(1, cv::Vec2d(x_in, y_in));
  std::vector<cv::Vec2d> dst(1);
  cv::undistortPoints(src, dst, K, dist_coeff);
  x_out = dst[0](0); y_out = dst[0](1);
  #endif
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

void cv::LocalizationSystem::precompute_undistort_map(void)
{
  undistort_map.create(height, width, CV_32FC2);
  for (int i = 0; i < height; i++) {
    std::vector<cv::Vec2f> coords_in(width);
    for (int j = 0; j < width; j++)
      coords_in[j] = cv::Vec2f(j,i); // TODO: reverse y? add 0.5?

    undistortPoints(coords_in, undistort_map.row(i), K, dist_coeff);
  }
}
