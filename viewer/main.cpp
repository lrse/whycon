#include <iostream>
#include <signal.h>
#include <boost/thread/thread.hpp>
#include "localization_viewer.h"
#include "localization_client.h"
using namespace std;

bool stop_process = false;
void interrupt(int s) {
  stop_process = true;
}

int main(void)
{
  signal(SIGINT, interrupt);
  signal(SIGTERM, interrupt);
  
  cv::LocalizationViewer viewer;
  cv::LocalizationClient client;
  
  viewer.start();
  client.start();
  
  while(!stop_process) {
    if (client.initialized) {
      cout << "updating" << endl;
      viewer.update(client.targets);
    }
    boost::this_thread::sleep(boost::posix_time::microseconds(1000));
  }

  viewer.stop();
}
