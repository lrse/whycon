#ifndef __LOCALIZATION_SERVICE_H__
#define __LOCALIZATION_SERVICE_H__

#include "config.h"

#ifdef ENABLE_MAVCONN

#include <mavconn.h>
#include <boost/thread/thread.hpp>

namespace cv {
  class LocalizationService {
    public:
      LocalizationService(void);
      ~LocalizationService(void);

      void start(void);

      lcm_t* lcm;
      mavconn_mavlink_msg_container_t_subscription_t* comm_sub;

    private:
      void lcm_wait(void);
      bool should_stop;
      boost::thread thread;
  };
}

#endif

#endif
