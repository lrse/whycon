#ifndef __LOCALIZATION_SERVICE_H__
#define __LOCALIZATION_SERVICE_H__

#include "config.h"

#ifdef ENABLE_MAVCONN

#include "mavlink/whycon/mavlink.h"
#include <mavconn.h>
#include <boost/thread/thread.hpp>
#include "localization_system.h"

namespace cv {
  class LocalizationService {
    public:
      LocalizationService(const cv::LocalizationSystem& system);
      ~LocalizationService(void);

      void start(void);
      void publish(void);

      lcm_t* lcm;
      mavconn_mavlink_msg_container_t_subscription_t* comm_sub;
      const cv::LocalizationSystem& system;

    private:
      void lcm_wait(void);
      bool should_stop;
      boost::thread thread;
      
  };
}

#endif

#endif
