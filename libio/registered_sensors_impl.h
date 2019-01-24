#ifndef REG_SENSORS_IMPL_H
#define REG_SENSORS_IMPL_H
#include "serial_protocol.h"
#include <cstring>
#include <iostream>
#include <string>
#include <stdint.h>

namespace serial_protocol {

// DECL
class Sensor_Default : public SensorBase
{
public:
  Sensor_Default(const unsigned int sen_len, const SensorType sensor_type);
  void publish();
  static SensorBase* Create(const unsigned int sen_len, const SensorType sensor_type) { return new Sensor_Default(sen_len, sensor_type); }
private:
  bool parse();
};


//IMPL
Sensor_Default::Sensor_Default(const unsigned int sen_len, const SensorType sensor_type) : SensorBase(sen_len, sensor_type)
  {
  }
void Sensor_Default::publish()
{
  if (previous_timestamp != timestamp)
  {
    previous_timestamp = timestamp;
    // printf something else there
    std::cout << "  raw data at " << timestamp << " :" ;
    for(unsigned int i = 0; i < len; i++)
    {
      std::cout << std::hex << ((char*)dataptr)[i] << " ";
      }
    std::cout <<  std::endl;
  }
}

bool Sensor_Default::parse()
{
  return true;
}

// DECL
class Sensor_IMU_MPU9250_Acc : public SensorBase
{
public:
  Sensor_IMU_MPU9250_Acc(const unsigned int sen_len, const SensorType sensor_type);
  void publish();
  static SensorBase* Create(const unsigned int sen_len, const SensorType sensor_type) { return new Sensor_IMU_MPU9250_Acc(sen_len, sensor_type); }
private:
  bool parse();
  float ax, ay, az;
};

// IMPL
  Sensor_IMU_MPU9250_Acc::Sensor_IMU_MPU9250_Acc(const unsigned int sen_len, const SensorType sensor_type) : SensorBase(sen_len, sensor_type)
  {
  }
void Sensor_IMU_MPU9250_Acc::publish()
{
  // printf something else there
  if (previous_timestamp != timestamp)
  {
    previous_timestamp = timestamp;
    // printf something else there
    std::cout << "  timestamp: " << timestamp << ", data ax: " << ax << ", ay:" << ay << ", az: " << az <<  std::endl;
  }
  
}

bool Sensor_IMU_MPU9250_Acc::parse()
{
  if(len >=12)
  {
    uint8_t* buf = (uint8_t*)get_data();
    if (buf)
    {
      memcpy(&ax, buf, sizeof(float));
      memcpy(&ay, buf+4, sizeof(float));
      memcpy(&az, buf+8, sizeof(float));
      return true;
    }
  }
  return false;
}


}
#endif
