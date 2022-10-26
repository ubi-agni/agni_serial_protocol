#include <signal.h>
#include <stdio.h>
#include <iostream>
#include <boost/program_options.hpp>
#include "libio/serial_protocol.h"
#include "libio/serial_com.h"
#include <unistd.h>

namespace po=boost::program_options;

po::options_description options("options");
void usage(char* argv[]) {
  std::cout << "usage: " << argv[0] << " [options]" << std::endl
        << options << std::endl;
}

bool handleCommandline(std::string &device, std::string &device_filename,
                       std::string &sensor_filename, int argc, char *argv[]) {
	// default input device
	device = "/dev/ttyACM0";

	// define processed options
	po::options_description inputs("input options");
	inputs.add_options()
		("serial,s", po::value<std::string>(&device)->implicit_value(device), "serial input device")
		("sensor_file", po::value<std::string>(&sensor_filename)->implicit_value(sensor_filename), "sensor type definition filename")
		("device_file", po::value<std::string>(&device_filename)->implicit_value(device_filename), "device type definition filename");
	options.add_options()
		("help,h", "Display this help message.");
	options.add(inputs);

	po::variables_map map;
	po::store(po::command_line_parser(argc, argv)
	          .options(options)
	          .run(), map);

	if (map.count("help")) return true;
	po::notify(map);
	return false;
}


bool bRun=true;
void mySigIntHandler(int sig) {
  bRun = false;
}

int main(int argc, char **argv)
{
  std::string sSerial;
  std::string sDeviceFilename;
  std::string sSensorFilename;

  // float ax,ay,az;

  // register Ctrl-C handler
  signal(SIGINT, mySigIntHandler);

  serial_protocol::SerialCom s;

  try
  {
    if (handleCommandline(sSerial, sDeviceFilename, sSensorFilename, argc, argv))
    {
      usage(argv);
      return EXIT_SUCCESS;
    }
  }
  catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    usage(argv);
    return EXIT_FAILURE;
  }
  try
  {
    std::cout << "connecting to " << sSerial << "\n";
    s.connect(sSerial);
    //s.connect("/tmp/ttyV0");
    s.setTimeOut(1000);
    //s.setVerbose(true);
    std::cout << "connected\n";
    std::cout << "creating serial protocol with  device_filename " << sDeviceFilename << "\n";
    serial_protocol::SerialProtocolBase p(&s, sDeviceFilename, sSensorFilename);
    //p.verbose = true;
    std::cout << "initializing serial protocol\n";
    if(p.init())
    {
      std::cout << "initialized\n";
      std::cout << "start streaming\n";
      p.start_streaming();

      std::cout << "streaming\n";
      while (bRun) // loop until Ctrl-C
      {
        try {
          p.update();
          //unsigned int timestamp = p.get_timestamp(1);
          //if(p.get_data_as_3_float(ax, ay, az, 1))
          //  std::cout << "  timestamp: " << timestamp << ", data ax: " << ax << ", ay:" << ay << ", az: " << az <<  std::endl;
          p.publish();
        } catch (const std::exception &e) {
            std::cerr << e.what() << std::endl;
          break;
        }
      }
      p.stop_streaming();
    }
    else
      std::cout << "initialization failed\n";
  }
  catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
  }
  std::cout << "disconnecting\n";
  s.disconnect();
  std::cout << "disconnected\n";
  return 0;
}
