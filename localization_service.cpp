#include <iostream>
#include "localization_service.h"
using namespace std;

static void mavlink_handler(const lcm_recv_buf_t* rbuf, const char* channel,
                              const mavconn_mavlink_msg_container_t* container, void* user);

cv::LocalizationService::LocalizationService(const cv::LocalizationSystem& _system) :  lcm(NULL), system(_system), should_stop(false)
{
	
}

cv::LocalizationService::~LocalizationService(void)
{
  if (lcm) {
    should_stop = true;
    thread.join();

    mavconn_mavlink_msg_container_t_unsubscribe(lcm, comm_sub);
    lcm_destroy(lcm);
  }
}

void cv::LocalizationService::start(void)
{
  lcm = lcm_create("udpm://");
	if (!lcm) throw std::runtime_error("Could not initialize LCM bus");

	comm_sub = mavconn_mavlink_msg_container_t_subscribe(lcm, MAVLINK_MAIN, &mavlink_handler, this);
  thread = boost::thread(&cv::LocalizationService::lcm_wait, this);

  cout << "started service" << endl;
}

void cv::LocalizationService::publish(void)
{
  cout << "publishing" << endl;
  for (int i = 0; i < system.targets; i++) {
    LocalizationSystem::Pose pose = (system.axis_set ? system.get_transformed_pose(i) : system.get_pose(i));
    mavlink_message_t msg;
    mavlink_msg_whycon_target_position_pack(getSystemID(), 0, &msg, i, 0, pose.pos(0), pose.pos(1), pose.pos(2));
    sendMAVLinkMessage(lcm, &msg);
  }
}

void cv::LocalizationService::lcm_wait(void)
{
  int fd = lcm_get_fileno(lcm);
  fd_set set;
  timeval t;
  t.tv_usec = 10000;
  t.tv_sec = 0;
  
	while(!should_stop) {
    FD_ZERO(&set);
    FD_SET(fd, &set);
    if (select(fd + 1, &set, NULL, NULL, &t) == -1)
      throw std::runtime_error("Localization service select() error: " + std::string(strerror(errno)));
      
    if (FD_ISSET(fd, &set)) {
      lcm_handle(lcm);
    }
  }
}

static void mavlink_handler(const lcm_recv_buf_t* rbuf, const char* channel,
                              const mavconn_mavlink_msg_container_t* container, void* user)
{
  const mavlink_message_t* msg = getMAVLinkMsgPtr(container);
	mavlink_message_t response;
  cv::LocalizationService* service = static_cast<cv::LocalizationService*>(user);
	lcm_t* lcm = service->lcm;

	switch(msg->msgid)
	{
    case MAVLINK_MSG_ID_WHYCON_GET_PARAMETERS:
    {
      cout << "sent parameters" << endl;
      mavlink_msg_whycon_parameters_pack(getSystemID(), 0, &response, service->system.targets);
      sendMAVLinkMessage(lcm, &response);
    }
    break;
    default:
      cerr << "Unhandled message " << (int)msg->msgid << " received from client" << endl;
		break;
	}
}
