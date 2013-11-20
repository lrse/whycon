#ifndef __LOCALIZATION_CLIENT_H__
#define __LOCALIZATION_CLIENT_H__

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include "../mavlink/whycon/mavlink.h"
#include <mavconn.h>
#include <vector>

namespace cv {
  struct Pose {
    float x, y, z;
  };

  class LocalizationClient {
    public:
      LocalizationClient(void);
      ~LocalizationClient(void);

      void start(void);

      lcm_t* lcm;
      mavconn_mavlink_msg_container_t_subscription_t* comm_sub;

      bool initialized;

      std::vector<Pose> targets;

    private:
      bool should_stop;
      
      void loop(void);
      void lcm_wait(void);
      boost::thread thread;
      boost::mutex mutex;
  };
}

#endif
