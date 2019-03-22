#ifndef REG_SENSORS_IMPL_H
#define REG_SENSORS_IMPL_H
#include "serial_protocol.h"
#include <cstring>
#include <iostream>
#include <string>
#include <stdint.h>
#ifdef HAVE_ROS
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Joy.h>
#endif

#define BIGENDIAN_TO_SIGNED_INT16(b)  (b)[0]*256 + (b)[1]
#define LITTLEENDIAN_TO_SIGNED_INT16(b)  (b)[1]*256 + (b)[0]
#define LITTLEENDIAN_TO_SIGNED_INT8(b)   (b)[0]

namespace serial_protocol {

// DECL
class Sensor_Default : public SensorBase
{
public:
  Sensor_Default(const unsigned int sen_len, const SensorType sensor_type);
  void publish();
  static SensorBase* Create(const unsigned int sen_len, const SensorType sensor_type) { return new Sensor_Default(sen_len, sensor_type); }
#ifdef HAVE_ROS
  void init_ros(ros::NodeHandle &nh);
#endif
private:
  bool parse();
};


//IMPL
Sensor_Default::Sensor_Default(const unsigned int sen_len, const SensorType sensor_type) : SensorBase(sen_len, sensor_type)
  {
  }
  

#ifdef HAVE_ROS
void Sensor_Default::init_ros(ros::NodeHandle &nh)
{

}
#endif

void Sensor_Default::publish()
{
  if (previous_timestamp != timestamp)
  {
    previous_timestamp = timestamp;
    // printf something else there
    /*
    std::cout << "  raw data at " << timestamp << " :" ;
    for(unsigned int i = 0; i < len; i++)
    {
      std::cout << std::hex << (int)((char*)dataptr)[i] << " ";
      }
    std::cout <<  std::endl;
    */
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
#ifdef HAVE_ROS
  void init_ros(ros::NodeHandle &nh);
#endif
private:
  bool parse();
  float ax, ay, az;
#ifdef HAVE_ROS
  sensor_msgs::Imu msg;
  ros::Publisher pub;
#endif
};

// IMPL
  Sensor_IMU_MPU9250_Acc::Sensor_IMU_MPU9250_Acc(const unsigned int sen_len, const SensorType sensor_type) : SensorBase(sen_len, sensor_type)
  {

  }

#ifdef HAVE_ROS
void Sensor_IMU_MPU9250_Acc::init_ros(ros::NodeHandle &nh)
{
    pub = nh.advertise<sensor_msgs::Imu>(sensor.name, 10);
    std::cout << "advertized a ros node for sensor " << sensor.name << std::endl;
}
#endif

void Sensor_IMU_MPU9250_Acc::publish()
{
  // printf something else there
  if (previous_timestamp != timestamp)
  {
    previous_timestamp = timestamp;
#ifdef HAVE_ROS
    msg.header.stamp = ros::Time::now();
    msg.linear_acceleration.x = ax;
    msg.linear_acceleration.y = ay;
    msg.linear_acceleration.z = az;
    pub.publish(msg);
#else
    // printf something else there
    std::cout << "  timestamp: " << timestamp << ", data ax: " << ax << ", ay:" << ay << ", az: " << az <<  std::endl;
#endif
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

/* Base IMU */

// DECL
class Sensor_IMU : public SensorBase
{
public:
  Sensor_IMU(const unsigned int sen_len, const SensorType sensor_type);
  void publish();
#ifdef HAVE_ROS
  void init_ros(ros::NodeHandle &nh);
#endif

protected:
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  float qw, qx, qy, qz;
private:
  virtual bool parse()=0;
#ifdef HAVE_ROS
  sensor_msgs::Imu msg;
  ros::Publisher pub;
#endif
};

// IMPL
Sensor_IMU::Sensor_IMU(const unsigned int sen_len, const SensorType sensor_type) : SensorBase(sen_len, sensor_type)
{

}

#ifdef HAVE_ROS
void Sensor_IMU::init_ros(ros::NodeHandle &nh)
{
    pub = nh.advertise<sensor_msgs::Imu>(sensor.name, 10);
    std::cout << "advertized a ros node for an IMU sensor " << sensor.name << std::endl;
}
#endif

void Sensor_IMU::publish()
{
  // printf something else there
  if (previous_timestamp != timestamp)
  {
    previous_timestamp = timestamp;
#ifdef HAVE_ROS
    msg.header.stamp = ros::Time::now();
    msg.linear_acceleration.x = ax;
    msg.linear_acceleration.y = ay;
    msg.linear_acceleration.z = az;
    msg.angular_velocity.x = gx;
    msg.angular_velocity.y = gy;
    msg.angular_velocity.z = gz;
    // no storage for mx, my, mz
    msg.orientation.w = qw;
    msg.orientation.x = qx;
    msg.orientation.y = qy;
    msg.orientation.z = qz;
    pub.publish(msg);
#else
    // printf something else there
    std::cout << "  timestamp: " << timestamp << ", data ax: " << ax << ", ay: " << ay << ", az: " << az <<  std::endl;
    std::cout << "  timestamp: " << timestamp << ", data gx: " << gx << ", gy: " << gy << ", gz: " << gz <<  std::endl;
    std::cout << "  timestamp: " << timestamp << ", data mx: " << mx << ", my: " << my << ", mz: " << mz <<  std::endl;
    std::cout << "  timestamp: " << timestamp << ", data qw: " << qw << ", qx: " << qx << ", qy: " << qy << ", qz: " << qz <<  std::endl;
#endif
  }
  
}

/* MPU9250 */

class Sensor_MPU9250 : public Sensor_IMU
{
public:
  Sensor_MPU9250(const unsigned int sen_len, const SensorType sensor_type);
  static SensorBase* Create(const unsigned int sen_len, const SensorType sensor_type) { return new Sensor_MPU9250(sen_len, sensor_type); }
private:
  bool parse();

};

Sensor_MPU9250::Sensor_MPU9250(const unsigned int sen_len, const SensorType sensor_type) : Sensor_IMU(sen_len, sensor_type)
{

}

bool Sensor_MPU9250::parse()
{
  if(len >=52)
  {
    uint8_t* buf = (uint8_t*)get_data();
    if (buf)
    {
      memcpy(&ax, buf, sizeof(float));
      memcpy(&ay, buf+4, sizeof(float));
      memcpy(&az, buf+8, sizeof(float));
      memcpy(&gx, buf+12, sizeof(float));
      memcpy(&gy, buf+16, sizeof(float));
      memcpy(&gz, buf+20, sizeof(float));
      memcpy(&mx, buf+24, sizeof(float));
      memcpy(&my, buf+28, sizeof(float));
      memcpy(&mz, buf+32, sizeof(float));
      memcpy(&qw, buf+36, sizeof(float));
      memcpy(&qx, buf+40, sizeof(float));
      memcpy(&qy, buf+44, sizeof(float));
      memcpy(&qz, buf+48, sizeof(float));
      return true;
    }
  }
  return false;
}


/* BNO055 */

class Sensor_BNO055 : public Sensor_IMU
{
public:
  Sensor_BNO055(const unsigned int sen_len, const SensorType sensor_type);
  static SensorBase* Create(const unsigned int sen_len, const SensorType sensor_type) { return new Sensor_BNO055(sen_len, sensor_type); }
private:
  bool parse();

};

Sensor_BNO055::Sensor_BNO055(const unsigned int sen_len, const SensorType sensor_type) : Sensor_IMU(sen_len, sensor_type)
{

}

bool Sensor_BNO055::parse()
{
  if(len >=26)
  {
    uint8_t* buf = (uint8_t*)get_data();
    if (buf)
    {
      
      // buffers are a sequences of signed 16bits integers in big-endian
      // order is qw,qx,qy,qz , ax, ay, az, mx, my, mz, gx, gy, gz
      signed short tmp = BIGENDIAN_TO_SIGNED_INT16(buf);
      qw = (float)tmp;
      tmp = BIGENDIAN_TO_SIGNED_INT16(buf+2);
      qx = (float)tmp;
      tmp = BIGENDIAN_TO_SIGNED_INT16(buf+4);
      qy = (float)tmp;
      tmp = BIGENDIAN_TO_SIGNED_INT16(buf+6);
      qz = (float)tmp;
      tmp = BIGENDIAN_TO_SIGNED_INT16(buf+8);
      ax = (float)tmp;
      tmp = BIGENDIAN_TO_SIGNED_INT16(buf+10);
      ay = (float)tmp;
      tmp = BIGENDIAN_TO_SIGNED_INT16(buf+12);
      az = (float)tmp;
      tmp = BIGENDIAN_TO_SIGNED_INT16(buf+14);
      mx = (float)tmp;
      tmp = BIGENDIAN_TO_SIGNED_INT16(buf+16);
      my = (float)tmp;
      tmp = BIGENDIAN_TO_SIGNED_INT16(buf+18);
      mz = (float)tmp;
      tmp = BIGENDIAN_TO_SIGNED_INT16(buf+20);
      gx = (float)tmp;
      tmp = BIGENDIAN_TO_SIGNED_INT16(buf+22);
      gy = (float)tmp;
      tmp = BIGENDIAN_TO_SIGNED_INT16(buf+24);
      gz = (float)tmp;
      return true;
    }
    double norm = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz );
    if (norm !=0)
    {
      qw /= norm;
      qx /= norm;
      qy /= norm;
      qz /= norm;
    }
  }
  return false;
}



/* BMA255 */

class Sensor_BMA255 : public Sensor_IMU
{
public:
  Sensor_BMA255(const unsigned int sen_len, const SensorType sensor_type);
  static SensorBase* Create(const unsigned int sen_len, const SensorType sensor_type) { return new Sensor_BMA255(sen_len, sensor_type); }
private:
  bool parse();

};

Sensor_BMA255::Sensor_BMA255(const unsigned int sen_len, const SensorType sensor_type) : Sensor_IMU(sen_len, sensor_type)
{
  qx = qy = qz = 0;
  qw = 1.0;
  mx = my = mz = 0;
  gx = gy = gz = 0;
}

bool Sensor_BMA255::parse()
{
  if(len >=6)
  {
    uint8_t* buf = (uint8_t*)get_data();
    if (buf)
    {
      // buffers are a sequences of signed 16bits integers in little-endian
      // order is ax, ay, az
      signed short tmp = LITTLEENDIAN_TO_SIGNED_INT16(buf);
      ax = (float)tmp;
      tmp = LITTLEENDIAN_TO_SIGNED_INT16(buf+2);
      ay = (float)tmp;
      tmp = LITTLEENDIAN_TO_SIGNED_INT16(buf+4);
      az = (float)tmp;
      return true;
    }
  }
  return false;
}


/* Base BARO */

// DECL
class Sensor_Baro : public SensorBase
{
public:
  Sensor_Baro(const unsigned int sen_len, const SensorType sensor_type);
  void publish();
#ifdef HAVE_ROS
  void init_ros(ros::NodeHandle &nh);
#endif

protected:
  float pressure;
private:
  virtual bool parse()=0;
#ifdef HAVE_ROS
  sensor_msgs::FluidPressure msg;
  ros::Publisher pub;
#endif
};

// IMPL
Sensor_Baro::Sensor_Baro(const unsigned int sen_len, const SensorType sensor_type) : SensorBase(sen_len, sensor_type)
{
}

#ifdef HAVE_ROS
void Sensor_Baro::init_ros(ros::NodeHandle &nh)
{
    pub = nh.advertise<sensor_msgs::FluidPressure>(sensor.name, 10);
    std::cout << "advertized a ros node for a Baro sensor " << sensor.name << std::endl;
}
#endif

void Sensor_Baro::publish()
{
  if (previous_timestamp != timestamp)
  {
    previous_timestamp = timestamp;
#ifdef HAVE_ROS
    msg.header.stamp = ros::Time::now();
    msg.fluid_pressure = pressure;
    pub.publish(msg);
#else
    // printf something else there
    std::cout << "  timestamp: " << timestamp << ", data pressure: " << pressure << std::endl;
#endif
  }
}


/* MPL115A2 */

class Sensor_MPL115A2 : public Sensor_Baro
{
public:
  Sensor_MPL115A2(const unsigned int sen_len, const SensorType sensor_type);
  static SensorBase* Create(const unsigned int sen_len, const SensorType sensor_type) { return new Sensor_MPL115A2(sen_len, sensor_type); }
private:
  bool parse();

};

Sensor_MPL115A2::Sensor_MPL115A2(const unsigned int sen_len, const SensorType sensor_type) : Sensor_Baro(sen_len, sensor_type)
{

}

bool Sensor_MPL115A2::parse()
{
  if(len >=4)
  {
    uint8_t* buf = (uint8_t*)get_data();
    if (buf)
    {
      memcpy(&pressure, buf, sizeof(float));
      return true;
    }
  }
  return false;
}


/* Base Joy */


// DECL
class Sensor_Joy : public SensorBase
{
public:
  Sensor_Joy(const unsigned int sen_len, const SensorType sensor_type);
  void publish();
#ifdef HAVE_ROS
  void init_ros(ros::NodeHandle &nh);
#endif

protected:
  std::vector<float> joy_pos;
  std::vector<int> joy_buttons;
private:
  virtual bool parse()=0;
#ifdef HAVE_ROS
  sensor_msgs::Joy msg;
  ros::Publisher pub;
#endif
};

// IMPL
Sensor_Joy::Sensor_Joy(const unsigned int sen_len, const SensorType sensor_type) : SensorBase(sen_len, sensor_type)
{
}

#ifdef HAVE_ROS
void Sensor_Joy::init_ros(ros::NodeHandle &nh)
{
    pub = nh.advertise<sensor_msgs::Joy>(sensor.name, 10);
    std::cout << "advertized a ros node for a Joy sensor " << sensor.name << std::endl;
}
#endif

void Sensor_Joy::publish()
{
  if (previous_timestamp != timestamp)
  {
    previous_timestamp = timestamp;
#ifdef HAVE_ROS
    msg.header.stamp = ros::Time::now();
    msg.axes = joy_pos;
    msg.buttons = joy_buttons;
    pub.publish(msg);
#else
    // printf something else there
    std::cout << "  timestamp: " << timestamp << "\n\taxes: ";
    for (size_t i=0; i < joy_pos.size(); i++)
    {
      std::cout << joy_pos[i] << " | ";
    }
    std::cout <<  std::endl;
    std::cout << "\tbuttons: ";
    for (size_t i=0; i < joy_buttons.size(); i++)
    {
      std::cout << joy_buttons[i] << " | ";
    }
    std::cout <<  std::endl;
#endif
  }
}


/* AS5013 */

class Sensor_AS5013 : public Sensor_Joy
{
public:
  Sensor_AS5013(const unsigned int sen_len, const SensorType sensor_type);
  static SensorBase* Create(const unsigned int sen_len, const SensorType sensor_type) { return new Sensor_AS5013(sen_len, sensor_type); }
private:
  bool parse();

};

Sensor_AS5013::Sensor_AS5013(const unsigned int sen_len, const SensorType sensor_type) : Sensor_Joy(sen_len, sensor_type)
{
  joy_pos.resize(2);
  joy_buttons.clear();
}

bool Sensor_AS5013::parse()
{
  if(len >=2)
  {
    uint8_t* buf = (uint8_t*)get_data();
    if (buf)
    {
      // buffers are a sequences of signed 8bits integers in little-endian
      // order is x, y
      signed char tmp = LITTLEENDIAN_TO_SIGNED_INT8(buf);
      joy_pos[0] = (float)tmp;
      tmp = LITTLEENDIAN_TO_SIGNED_INT8(buf+1);
      joy_pos[1] = (float)tmp;
      return true;
    }
  }
  return false;
}

/* AS5013 */

class Sensor_AS5013y : public Sensor_Joy
{
public:
  Sensor_AS5013y(const unsigned int sen_len, const SensorType sensor_type);
  static SensorBase* Create(const unsigned int sen_len, const SensorType sensor_type) { return new Sensor_AS5013y(sen_len, sensor_type); }
private:
  bool parse();

};

Sensor_AS5013y::Sensor_AS5013y(const unsigned int sen_len, const SensorType sensor_type) : Sensor_Joy(sen_len, sensor_type)
{
  joy_pos.resize(2);
  joy_pos[0] = 0;
  joy_buttons.clear();
}

bool Sensor_AS5013y::parse()
{
  if(len >=1)
  {
    uint8_t* buf = (uint8_t*)get_data();
    if (buf)
    {
      // buffers are a sequences of signed 8bits integers in little-endian
      // order is y
      signed char tmp = LITTLEENDIAN_TO_SIGNED_INT8(buf);
      joy_pos[1] = (float)tmp;
      return true;
    }
  }
  return false;
}


}
#endif
