#include "localization_viewer.h"
#ifdef ENABLE_VIEWER

cv::LocalizationViewer::LocalizationViewer(const cv::LocalizationSystem& _system) : visualizer(NULL), system(_system),
  changed_point_cloud(false)
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

  // targets point cloud
  point_cloud = pcl::PointCloud<pcl::PointXYZ>().makeShared();
  point_cloud->resize(system.targets);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(point_cloud, 0, 255, 0);
  visualizer->addPointCloud<pcl::PointXYZ>(point_cloud, cloud_color, "targets");
  visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "targets");
  
  thread = boost::thread(&cv::LocalizationViewer::loop, this);
}

cv::LocalizationViewer::~LocalizationViewer(void) {
  if (visualizer) {      
    visualizer->close();
    thread.join();
    delete visualizer;
  }
}

void cv::LocalizationViewer::loop(void)
{
  while (!visualizer->wasStopped()) {
    if (changed_point_cloud) {
      boost::mutex::scoped_lock m(mutex);
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(point_cloud, 0, 255, 0);
      visualizer->updatePointCloud(point_cloud, cloud_color, "targets");
      changed_point_cloud = false;
    }
    visualizer->spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }
  visualizer->close();
}

void cv::LocalizationViewer::update(void)
{
  boost::mutex::scoped_lock m(mutex);
  for (int i = 0; i < point_cloud->size(); i++) {
    cv::LocalizationSystem::Pose pose = system.get_transformed_pose(i); // TODO: use stored transformed poses to reduce overhead of this calld
    (*point_cloud)[i] = pcl::PointXYZ(pose.pos(0), pose.pos(1), pose.pos(2));
    changed_point_cloud = true;
  }
}

void cv::LocalizationViewer::wait(void)
{
  if (visualizer) thread.join();
}

#endif
