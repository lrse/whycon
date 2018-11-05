#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "whycon_ros.h"

namespace whycon {
  class WhyconNodelet : public nodelet::Nodelet
  {
    public:
      void onInit(void) {
        t = boost::make_shared<WhyConROS>(getPrivateNodeHandle());
      }

      boost::shared_ptr<WhyConROS> t;
  };
}

PLUGINLIB_EXPORT_CLASS(whycon::WhyconNodelet, nodelet::Nodelet)


