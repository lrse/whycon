#include <sstream>
#include "localization_viewer.h"
using namespace std;

cv::LocalizationViewer::LocalizationViewer(void) : should_stop(false),
  visualizer(NULL), updated(false)
{
}

void cv::LocalizationViewer::start(void) {
  visualizer = new pcl::visualization::PCLVisualizer("3D view");
  
  cout << "starting viewer" << endl;
  visualizer->setBackgroundColor(0.0, 0.0, 0.0);

  // add XY ground plane
  pcl::ModelCoefficients plane_coefficients;
  plane_coefficients.values.resize(4);
  plane_coefficients.values[0] = 0;
  plane_coefficients.values[1] = 0;
  plane_coefficients.values[2] = 1;
  plane_coefficients.values[3] = 0;
  visualizer->addPlane(plane_coefficients);
  visualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.2, 0.2, 0.2, "plane");
  
  thread = boost::thread(&cv::LocalizationViewer::loop, this);
}

cv::LocalizationViewer::~LocalizationViewer(void) {
  if (visualizer) {      
    visualizer->close();
    thread.join();
    delete visualizer;
  }
}

void cv::LocalizationViewer::stop(void) {
  should_stop = true;
}

void cv::LocalizationViewer::loop(void)
{
  while (!visualizer->wasStopped() && !should_stop) {
    if (updated) {
      boost::mutex::scoped_lock m(mutex);
      pcl::ModelCoefficients cylinder_coeff;
      cylinder_coeff.values.resize(7);
      cylinder_coeff.values[6] = 0.01;
      for (int i = 0; i < targets.size(); i++) {
        cylinder_coeff.values[0] = targets[i].x;
        cylinder_coeff.values[1] = targets[i].y;
        cylinder_coeff.values[2] = targets[i].z;
        cylinder_coeff.values[3] = 0;
        cylinder_coeff.values[4] = 0;
        cylinder_coeff.values[5] = 0.01;
        ostringstream ostr;
        ostr << "target" << i;
        visualizer->addCylinder(cylinder_coeff, ostr.str());
        cout << "added cyl" << ostr.str() << " at " << targets[i].x << " " << targets[i].y << " " << targets[i].z << endl;
      }
      visualizer->setRepresentationToSurfaceForAllActors();
      // do something
      updated = false;
    }
    visualizer->spinOnce(10);
    boost::this_thread::sleep(boost::posix_time::microseconds(10000));

    for (int i = 0; i < targets.size(); i++) {
      ostringstream ostr;
      ostr << "target" << i;
      visualizer->removeShape(ostr.str());
      cout << "removed cyl" << ostr.str() << endl;
    }
  }
  visualizer->close();
}

void cv::LocalizationViewer::update(const std::vector<Pose>& _targets)
{
  boost::mutex::scoped_lock m(mutex);
  targets = _targets;
  updated = true;
  cout << "updated" << endl;
}

