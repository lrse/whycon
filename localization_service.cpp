#include "localization_service.h"

static void mavlink_handler(const lcm_recv_buf_t* rbuf, const char* channel,
                              const mavconn_mavlink_msg_container_t* container, void* user);

cv::LocalizationService::LocalizationService(void) : lcm(NULL), should_stop(false)
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

	comm_sub = mavconn_mavlink_msg_container_t_subscribe(lcm, MAVLINK_MAIN, &mavlink_handler, lcm);
  thread = boost::thread(&cv::LocalizationService::lcm_wait, this);  
}

void cv::LocalizationService::lcm_wait(void)
{
	while(!should_stop) lcm_handle(lcm);
}

static void mavlink_handler(const lcm_recv_buf_t* rbuf, const char* channel,
                              const mavconn_mavlink_msg_container_t* container, void* user)
{
  const mavlink_message_t* msg = getMAVLinkMsgPtr(container);
	mavlink_message_t response;
	lcm_t* lcm = static_cast<cv::LocalizationService*>(user)->lcm;
	//printf("Received message #%d on channel \"%s\" (sys:%d|comp:%d):\n", msg->msgid, channel, msg->sysid, msg->compid);

#if 0
	switch(msg->msgid)
	{
	uint32_t receiveTime;
	uint32_t sendTime;
	case MAVLINK_MSG_ID_COMMAND_LONG:
	{
		mavlink_command_long_t cmd;
		mavlink_msg_command_long_decode(msg, &cmd);
		printf("Message ID: %d\n", msg->msgid);
		printf("Command ID: %d\n", cmd.command);
		printf("Target System ID: %d\n", cmd.target_system);
		printf("Target Component ID: %d\n", cmd.target_component);
		printf("\n");

		if (cmd.confirmation)
		{
			printf("Confirmation requested, sending confirmation:\n");
			mavlink_command_ack_t ack;
			ack.command = cmd.command;
			ack.result = 3;
			mavlink_msg_command_ack_encode(getSystemID(), compid, &response, &ack);
			sendMAVLinkMessage(lcm, &response);
		}
	}
		break;
	case MAVLINK_MSG_ID_ATTITUDE:
		gettimeofday(&tv, NULL);
		receiveTime = tv.tv_usec;
		sendTime = mavlink_msg_attitude_get_time_boot_ms(msg);
		printf("Received attitude message, transport took %f ms\n", (receiveTime - sendTime)/1000.0f);
		break;
	case MAVLINK_MSG_ID_GPS_RAW_INT:
	{
		mavlink_gps_raw_int_t gps;
		mavlink_msg_gps_raw_int_decode(msg, &gps);
		printf("GPS: lat: %f, lon: %f, alt: %f\n", gps.lat/(double)1E7, gps.lon/(double)1E7, gps.alt/(double)1E6);
		break;
	}
	case MAVLINK_MSG_ID_RAW_PRESSURE:
	{
		mavlink_raw_pressure_t p;
		mavlink_msg_raw_pressure_decode(msg, &p);
		printf("PRES: %f\n", p.press_abs/(double)1000);
	}
	break;
	default:
		printf("ERROR: could not decode message with ID: %d\n", msg->msgid);
		break;
	}
  #endif  
}
