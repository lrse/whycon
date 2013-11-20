#include <iostream>
#include "localization_client.h"
using namespace std;

static void mavlink_handler(const lcm_recv_buf_t* rbuf, const char* channel,
                              const mavconn_mavlink_msg_container_t* container, void* user);


cv::LocalizationClient::LocalizationClient(void) : lcm(NULL), initialized(false), should_stop(false)
{
}

void cv::LocalizationClient::start(void)
{
  lcm = lcm_create("udpm://");
	if (!lcm) throw std::runtime_error("Could not initialize LCM bus");

	comm_sub = mavconn_mavlink_msg_container_t_subscribe(lcm, MAVLINK_MAIN, &mavlink_handler, this);

  // ask for parameters
  mavlink_message_t msg;
  mavlink_msg_whycon_get_parameters_pack(getSystemID(), 0, &msg, -1);
  sendMAVLinkMessage(lcm, &msg);

  // start read loop
  thread = boost::thread(&cv::LocalizationClient::lcm_wait, this);
}

cv::LocalizationClient::~LocalizationClient(void)
{
  if (lcm) {
    should_stop = true;
    thread.join();

    mavconn_mavlink_msg_container_t_unsubscribe(lcm, comm_sub);
    lcm_destroy(lcm);
  }
}

void cv::LocalizationClient::lcm_wait(void)
{
  int fd = lcm_get_fileno(lcm);
  timeval t;
  t.tv_usec = 10000;
  t.tv_sec = 0;
  
	while(!should_stop) {
    fd_set set;
    FD_ZERO(&set);
    FD_SET(fd, &set);
    if (select(fd+1, &set, NULL, NULL, &t) == -1)
      throw std::runtime_error("Localization client select() error: " + std::string(strerror(errno)));
      
    if (FD_ISSET(fd, &set)) {
      lcm_handle(lcm);
    }
  }
}

static void mavlink_handler(const lcm_recv_buf_t* rbuf, const char* channel,
                              const mavconn_mavlink_msg_container_t* container, void* user)
{
  const mavlink_message_t* msg = getMAVLinkMsgPtr(container);
	cv::LocalizationClient* client = (cv::LocalizationClient*)user;
  //lcm_t* lcm = client->lcm;

	switch(msg->msgid)
	{
    case MAVLINK_MSG_ID_WHYCON_PARAMETERS:
    {
      cout << "received parameters" << endl;
      mavlink_whycon_parameters_t parameters;
      mavlink_msg_whycon_parameters_decode(msg, &parameters);
      client->targets.resize(parameters.targets);
      client->initialized = true;
    }
    break;
    case MAVLINK_MSG_ID_WHYCON_TARGET_POSITION:
    {
      if (!client->initialized) break; // get parameters instead?
      mavlink_whycon_target_position_t position;
      mavlink_msg_whycon_target_position_decode(msg, &position);
      cv::Pose& pose = client->targets[position.id];
      pose.x = position.x;
      pose.y = position.y;
      pose.z = position.z;
      cout << "received target position" << endl;
    }
    break;
    default:
      cerr << "Unhandled message " << (int)msg->msgid << " received from server" << endl;
		break;
	}
}
