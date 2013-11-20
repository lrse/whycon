#ifndef __LOCALIZATION_VIEWER_H__
#define __LOCALIZATION_VIEWER_H__

#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include "localization_client.h"

namespace cv {
  class LocalizationViewer {
    public:
      LocalizationViewer(void);
      ~LocalizationViewer(void);

      void start(void);
      void update(const std::vector<Pose>& targets);
      void stop(void);
      

    private:
      //void init_visualization(pcl::visualization::PCLVisualizer& viewer);

      bool should_stop;

      pcl::visualization::PCLVisualizer* visualizer;
      std::vector<Pose> targets;
      bool updated;
      
      void loop(void);
      boost::thread thread;
      boost::mutex mutex;
  };
}

#endif
