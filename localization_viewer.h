#ifndef __LOCALIZATION_VIEWER_H__
#define __LOCALIZATION_VIEWER_H__

#include "config.h"

#ifdef ENABLE_VIEWER

#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include "localization_system.h"

namespace cv {
  class LocalizationViewer {
    public:
      LocalizationViewer(const LocalizationSystem& system);
      ~LocalizationViewer(void);

      void start(void);
      void update(void);
      void wait(void);
      void stop(void);
      

    private:
      //void init_visualization(pcl::visualization::PCLVisualizer& viewer);

      bool should_stop;

      pcl::visualization::PCLVisualizer* visualizer;
      pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud;
      const LocalizationSystem& system;
      bool changed_point_cloud;
      
      void loop(void);
      boost::thread thread;
      boost::mutex mutex;
  };
}

#endif

#endif
