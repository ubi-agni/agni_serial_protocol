#include <signal.h>
#include <stdio.h>
#include <iostream>
#include <curses.h>
#include "libio/serial_protocol.h"
#include "libio/serial_com.h"
#include <unistd.h>

bool bRun=true;
void mySigIntHandler(int sig) {
  bRun = false;
}

int main(int argc, char **argv)
{
  // register Ctrl-C handler
  signal(SIGINT, mySigIntHandler);
  
  serial_protocol::SerialCom s;
  try
  {
    std::cout << "connecting to /dev/ttyACM0\n";
    s.connect("/dev/ttyACM0");
    //s.connect("/tmp/ttyV0");
    s.setTimeOut(1000);
    //s.setVerbose(true);
    std::cout << "connected\n";  
    serial_protocol::SerialProtocolBase p(&s);
    std::cout << "initializing serial protocol\n";
    if(p.init())
    {
      std::cout << "initialized\n";
      std::cout << "start streaming\n"; 
      p.start_streaming();
      //p.verbose = true;
      std::cout << "streaming\n"; 
      unsigned char ch=0;
      while (bRun && ch != 'q') // loop until Ctrl-C
      {
        try {
          p.loop();
          unsigned int timestamp = p.get_timestamp(1);
          double val = p.get_data_as_double(1);
          std::cout << "  timestamp: " << timestamp << ", data : " << val << "" << std::endl;

        } catch (const std::exception &e) {
            std::cerr << e.what() << std::endl;
          break;
        }
        ch = getch();
      }
      p.stop_streaming();
    }
    else
      std::cout << "initialization failed\n";
  }
  catch (const std::exception &e) {

    std::cerr << e.what() << std::endl;
    return -1;
  }
  std::cout << "disconnecting\n";
  s.disconnect();
  std::cout << "disconnected\n";
  return 0;
}
