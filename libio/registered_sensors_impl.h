#ifndef REG_SENSORS_IMPL_H
#define REG_SENSORS_IMPL_H
#include "serial_protocol.h"
#include <cstring>
#include <iostream>
#include <string>
#include <sstream>
#include <stdint.h>
#ifdef HAVE_ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <tactile_msgs/TactileState.h>
#endif

#define BIGENDIAN_TO_SIGNED_INT16(b) (b)[0] * 256 + (b)[1]
#define BIGENDIAN_TO_UNSIGNED_INT16(b) static_cast<uint16_t>((b)[0]) * 256 + static_cast<uint16_t>((b)[1])
#define LITTLEENDIAN_TO_SIGNED_INT16(b) (b)[1] * 256 + (b)[0]
#define LITTLEENDIANUINT_TO_UNSIGNED_INT16(b) static_cast<uint16_t>((b)[1] * 256 + (b)[0])
#define TO_SIGNED_INT8(b) (b)[0]
#define TO_UNSIGNED_INT8(b) static_cast<uint8_t>((b)[0])
#define LITTLEENDIAN12_TO_UNSIGNED_INT16(b) static_cast<uint16_t>(((b)[1] & 0x0F)) * 256 + static_cast<uint16_t>((b)[0])
#define BIGENDIAN12_TO_UNSIGNED_INT16(b) static_cast<uint16_t>(((b)[0] & 0x0F)) * 256 + static_cast<uint16_t>((b)[1])
#define LITTLEENDIAN4MSB_TO_UNSIGNED_INT8(b) static_cast<uint8_t>(((b)[1] & 0xF0) >> 4)

namespace serial_protocol
{
// DECL

class Sensor_Default : public SensorBase
{
public:
  Sensor_Default(const uint16_t sen_len, const SensorType sensor_type);
  void publish();
  static SensorBase* Create(const uint16_t sen_len, const SensorType sensor_type)
  {
    return new Sensor_Default(sen_len, sensor_type);
  }
#ifdef HAVE_ROS
  void init_ros(ros::NodeHandle& nh);
#endif
protected:
  unsigned int sensor_id;

private:
  bool parse();
  std::stringstream sstr;
#ifdef HAVE_ROS
  std_msgs::String msg;
  ros::Publisher pub;
#endif
  static unsigned int sensor_count;
};

unsigned int Sensor_Default::sensor_count{ 0 };
// IMPL
Sensor_Default::Sensor_Default(const uint16_t sen_len, const SensorType sensor_type) : SensorBase(sen_len, sensor_type)
{
  sensor_id = ++sensor_count;
}

#ifdef HAVE_ROS
void Sensor_Default::init_ros(ros::NodeHandle& nh)
{
  std::stringstream sstr;
  if (sensor_id > 1)
    sstr << sensor.name << "_" << sensor_id;
  else
    sstr << sensor.name;
  pub = nh.advertise<std_msgs::String>(sstr.str(), 10);
  std::cout << "advertized a ros node for sensor " << sstr.str() << std::endl;
}
#endif

void Sensor_Default::publish()
{
  if (previous_timestamp != timestamp)
  {
    new_data = false;
    previous_timestamp = timestamp;
#ifdef HAVE_ROS
    std::stringstream sstream;
    sstream << "[" << ros::Time::now() << "], timestamp: " << timestamp << ", data:" << sstr.str();
    msg.data = sstream.str();
    pub.publish(msg);
#else
    // printf something else there
    std::cout << "  timestamp: " << timestamp << ", " << sstr.str() << std::endl;
#endif
  }
}

bool Sensor_Default::parse()
{
  uint8_t* buf = (uint8_t*)get_data();
  if (buf)
  {
    sstr.clear();
    sstr.str("");
    for (unsigned int i = 0; i < len; i++)
    {
      sstr << std::hex << (int)((uint8_t*)buf)[i] << "|";
    }
    new_data = true;
    return true;
  }
  else
    return false;
}

// DECL
class Sensor_IMU_MPU9250_Acc : public SensorBase
{
public:
  Sensor_IMU_MPU9250_Acc(const uint16_t sen_len, const SensorType sensor_type);
  void publish();
  static SensorBase* Create(const uint16_t sen_len, const SensorType sensor_type)
  {
    return new Sensor_IMU_MPU9250_Acc(sen_len, sensor_type);
  }
#ifdef HAVE_ROS
  void init_ros(ros::NodeHandle& nh);
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
Sensor_IMU_MPU9250_Acc::Sensor_IMU_MPU9250_Acc(const uint16_t sen_len, const SensorType sensor_type)
  : SensorBase(sen_len, sensor_type)
{
}

#ifdef HAVE_ROS
void Sensor_IMU_MPU9250_Acc::init_ros(ros::NodeHandle& nh)
{
  pub = nh.advertise<sensor_msgs::Imu>(sensor.name, 10);
  std::cout << "advertized a ros node for sensor " << sensor.name << std::endl;
}
#endif

void Sensor_IMU_MPU9250_Acc::publish()
{
  // printf something else there
  if (previous_timestamp != timestamp && new_data)
  {
    new_data = false;
    previous_timestamp = timestamp;
#ifdef HAVE_ROS
    msg.header.stamp = ros::Time::now();
    msg.linear_acceleration.x = ax;
    msg.linear_acceleration.y = ay;
    msg.linear_acceleration.z = az;
    pub.publish(msg);
#else
    // printf something else there
    std::cout << "  timestamp: " << timestamp << ", data ax: " << ax << ", ay:" << ay << ", az: " << az << std::endl;
#endif
  }
}

bool Sensor_IMU_MPU9250_Acc::parse()
{
  if (len >= 12)
  {
    uint8_t* buf = (uint8_t*)get_data();
    if (buf)
    {
      memcpy(&ax, buf, sizeof(float));
      memcpy(&ay, buf + 4, sizeof(float));
      memcpy(&az, buf + 8, sizeof(float));
      new_data = true;
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
  Sensor_IMU(const uint16_t sen_len, const SensorType sensor_type);
  void publish();
#ifdef HAVE_ROS
  void init_ros(ros::NodeHandle& nh);
#endif

protected:
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  float qw, qx, qy, qz;

private:
  virtual bool parse() = 0;
#ifdef HAVE_ROS
  sensor_msgs::Imu msg;
  ros::Publisher pub;
#endif
  static unsigned int sensor_count;
  unsigned int sensor_id;
};

// IMPL
Sensor_IMU::Sensor_IMU(const uint16_t sen_len, const SensorType sensor_type) : SensorBase(sen_len, sensor_type)
{
}

#ifdef HAVE_ROS
void Sensor_IMU::init_ros(ros::NodeHandle& nh)
{
  msg.header.frame_id = sensor.name;
  pub = nh.advertise<sensor_msgs::Imu>(sensor.name, 10);
  std::cout << "advertized a ros node for an IMU sensor " << sensor.name << std::endl;
}
#endif

void Sensor_IMU::publish()
{
  // printf something else there
  if (previous_timestamp != timestamp && new_data)
  {
    new_data = false;
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
    std::cout << "  timestamp: " << timestamp << ", data ax: " << ax << ", ay: " << ay << ", az: " << az << std::endl;
    std::cout << "  timestamp: " << timestamp << ", data gx: " << gx << ", gy: " << gy << ", gz: " << gz << std::endl;
    std::cout << "  timestamp: " << timestamp << ", data mx: " << mx << ", my: " << my << ", mz: " << mz << std::endl;
    std::cout << "  timestamp: " << timestamp << ", data qw: " << qw << ", qx: " << qx << ", qy: " << qy
              << ", qz: " << qz << std::endl;
#endif
  }
}

/* MPU9250 */

class Sensor_MPU9250 : public Sensor_IMU
{
public:
  Sensor_MPU9250(const uint16_t sen_len, const SensorType sensor_type);
  static SensorBase* Create(const uint16_t sen_len, const SensorType sensor_type)
  {
    return new Sensor_MPU9250(sen_len, sensor_type);
  }

private:
  bool parse();
};

Sensor_MPU9250::Sensor_MPU9250(const uint16_t sen_len, const SensorType sensor_type) : Sensor_IMU(sen_len, sensor_type)
{
}

bool Sensor_MPU9250::parse()
{
  if (len >= 52)
  {
    uint8_t* buf = (uint8_t*)get_data();
    if (buf)
    {
      memcpy(&ax, buf, sizeof(float));
      memcpy(&ay, buf + 4, sizeof(float));
      memcpy(&az, buf + 8, sizeof(float));
      memcpy(&gx, buf + 12, sizeof(float));
      memcpy(&gy, buf + 16, sizeof(float));
      memcpy(&gz, buf + 20, sizeof(float));
      memcpy(&mx, buf + 24, sizeof(float));
      memcpy(&my, buf + 28, sizeof(float));
      memcpy(&mz, buf + 32, sizeof(float));
      memcpy(&qw, buf + 36, sizeof(float));
      memcpy(&qx, buf + 40, sizeof(float));
      memcpy(&qy, buf + 44, sizeof(float));
      memcpy(&qz, buf + 48, sizeof(float));
      new_data = true;
      return true;
    }
  }
  return false;
}

/* BNO055 */

class Sensor_BNO055 : public Sensor_IMU
{
public:
  Sensor_BNO055(const uint16_t sen_len, const SensorType sensor_type);
  static SensorBase* Create(const uint16_t sen_len, const SensorType sensor_type)
  {
    return new Sensor_BNO055(sen_len, sensor_type);
  }

private:
  bool parse();
};

Sensor_BNO055::Sensor_BNO055(const uint16_t sen_len, const SensorType sensor_type) : Sensor_IMU(sen_len, sensor_type)
{
}

bool Sensor_BNO055::parse()
{
  if (len >= 26)
  {
    uint8_t* buf = (uint8_t*)get_data();
    if (buf)
    {
      // buffers are a sequences of signed 16bits integers in big-endian
      // order is qw,qx,qy,qz , ax, ay, az, mx, my, mz, gx, gy, gz
      /*signed short tmp = BIGENDIAN_TO_SIGNED_INT16(buf);
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
      gz = (float)tmp;*/

      signed short tmp = LITTLEENDIAN_TO_SIGNED_INT16(buf);
      qw = static_cast<float>(tmp);
      tmp = LITTLEENDIAN_TO_SIGNED_INT16(buf + 2);
      qx = static_cast<float>(tmp);
      tmp = LITTLEENDIAN_TO_SIGNED_INT16(buf + 4);
      qy = static_cast<float>(tmp);
      tmp = LITTLEENDIAN_TO_SIGNED_INT16(buf + 6);
      qz = static_cast<float>(tmp);
      tmp = LITTLEENDIAN_TO_SIGNED_INT16(buf + 8);
      ax = static_cast<float>(tmp);
      tmp = LITTLEENDIAN_TO_SIGNED_INT16(buf + 10);
      ay = static_cast<float>(tmp);
      tmp = LITTLEENDIAN_TO_SIGNED_INT16(buf + 12);
      az = static_cast<float>(tmp);
      tmp = LITTLEENDIAN_TO_SIGNED_INT16(buf + 14);
      mx = static_cast<float>(tmp);
      tmp = LITTLEENDIAN_TO_SIGNED_INT16(buf + 16);
      my = static_cast<float>(tmp);
      tmp = LITTLEENDIAN_TO_SIGNED_INT16(buf + 18);
      mz = static_cast<float>(tmp);
      tmp = LITTLEENDIAN_TO_SIGNED_INT16(buf + 20);
      gx = static_cast<float>(tmp);
      tmp = LITTLEENDIAN_TO_SIGNED_INT16(buf + 22);
      gy = static_cast<float>(tmp);
      tmp = LITTLEENDIAN_TO_SIGNED_INT16(buf + 24);
      gz = static_cast<float>(tmp);

      if (gx == 1280.0)
      {
        std::cout << "wrong IMU data. float " << std::endl;

        /*std::cout << "wrong IMU data. float val qw " << std::dec << qw << " hex 0x" << std::hex <<
        static_cast<int>(*(buf+0)) << "| 0x" << std::hex  << static_cast<int>(*(buf+1)) << std::endl;
        std::cout << "wrong IMU data. float val qx " << std::dec << qx << " hex 0x" << std::hex <<
        static_cast<int>(*(buf+2)) << "| 0x" << std::hex  << static_cast<int>(*(buf+3)) << std::endl;
        std::cout << "wrong IMU data. float val qy " << std::dec << qy << " hex 0x" << std::hex <<
        static_cast<int>(*(buf+4)) << "| 0x" << std::hex  << static_cast<int>(*(buf+5)) << std::endl;
        std::cout << "wrong IMU data. float val qz " << std::dec << qz << " hex 0x" << std::hex <<
        static_cast<int>(*(buf+6)) << "| 0x" << std::hex  << static_cast<int>(*(buf+7)) << std::endl;

        std::cout << "wrong IMU data. float val ax " << std::dec << ax << " hex 0x" << std::hex <<
        static_cast<int>(*(buf+8)) << "| 0x" << std::hex  << static_cast<int>(*(buf+9)) << std::endl;
        std::cout << "wrong IMU data. float val ay " << std::dec << ay << " hex 0x" << std::hex <<
        static_cast<int>(*(buf+10)) << "| 0x" << std::hex  << static_cast<int>(*(buf+11)) << std::endl;
        std::cout << "wrong IMU data. float val az " << std::dec << az << " hex 0x" << std::hex <<
        static_cast<int>(*(buf+12)) << "| 0x" << std::hex  << static_cast<int>(*(buf+13)) << std::endl;

        std::cout << "wrong IMU data. float val mx " << std::dec << mx << " hex 0x" << std::hex <<
        static_cast<int>(*(buf+14)) << "| 0x" << std::hex  << static_cast<int>(*(buf+15)) << std::endl;
        std::cout << "wrong IMU data. float val my " << std::dec << my << " hex 0x" << std::hex <<
        static_cast<int>(*(buf+16)) << "| 0x" << std::hex  << static_cast<int>(*(buf+17)) << std::endl;
        std::cout << "wrong IMU data. float val mz " << std::dec << mz << " hex 0x" << std::hex <<
        static_cast<int>(*(buf+18)) << "| 0x" << std::hex  << static_cast<int>(*(buf+19)) << std::endl;

        std::cout << "wrong IMU data. float val gx " << std::dec << gx << " hex 0x" << std::hex <<
        static_cast<int>(*(buf+20)) << "| 0x" << std::hex  << static_cast<int>(*(buf+21)) << std::endl;
        std::cout << "wrong IMU data. float val gy " << std::dec << gy << " hex 0x" << std::hex <<
        static_cast<int>(*(buf+22)) << "| 0x" << std::hex  << static_cast<int>(*(buf+23)) << std::endl;
        std::cout << "wrong IMU data. float val gz " << std::dec << gz << " hex 0x" << std::hex <<
        static_cast<int>(*(buf+24)) << "| 0x" << std::hex  << static_cast<int>(*(buf+25)) << std::endl;
        */
      }
      /*else
       std::cout << "correct IMU data. float " << std::endl;


     std::cout << "quat " << std::dec << qw << "," << qx << "," << qy << "," << qz << std::endl;
      std::cout << "acc "  << std::dec << ax << "," << ay << "," << az << std::endl;
      std::cout << "mag "  << std::dec << mx << "," << my << "," << mz << std::endl;
      std::cout << "gyr "   << std::dec << gx << "," << gy << "," << gz << std::endl;

      std::cout << "  raw data 0x" ;
      for(unsigned int i = 0; i < 26; i++)
      {
        std::cout << std::hex << static_cast<int>(*(buf+i)) << "|";
      }
      std::cout <<  std::endl;
      std::cout <<  std::endl;
*/
      // protect from bad data
      if (qx == -23131.0 && qy == -23131.0 && ax == -23131.0)
      {
        std::cout << "wrong IMU data 0xA5 0xA5 ... 0xA5 " << std::endl;
        qx = qy = qz = ax = ay = az = mx = my = mz = gx = gy = gz = 0;
        qw = 1;
        new_data = false;
        return false;
      }
      double norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
      if (norm != 0)
      {
        qw /= norm;
        qx /= norm;
        qy /= norm;
        qz /= norm;
      }
      new_data = true;
      return true;
    }
  }
  return false;
}

/* BNO08X */

class Sensor_BNO08X : public Sensor_IMU
{
public:
  Sensor_BNO08X(const uint16_t sen_len, const SensorType sensor_type);
  static SensorBase* Create(const uint16_t sen_len, const SensorType sensor_type)
  {
    return new Sensor_BNO08X(sen_len, sensor_type);
  }

private:
  bool parse();
};

Sensor_BNO08X::Sensor_BNO08X(const uint16_t sen_len, const SensorType sensor_type) : Sensor_IMU(sen_len, sensor_type)
{
}

bool Sensor_BNO08X::parse()
{
  if (len >= 40)
  {
    uint8_t* buf = (uint8_t*)get_data();
    if (buf)
    {
      // buffers are a sequences of floats in little-endian
      // order is ax, ay, az, gx, gy, gz, qw, qx, qy, qz
      memcpy(&ax, buf, sizeof(float));
      memcpy(&ay, buf + 4, sizeof(float));
      memcpy(&az, buf + 8, sizeof(float));
      memcpy(&gx, buf + 12, sizeof(float));
      memcpy(&gy, buf + 16, sizeof(float));
      memcpy(&gz, buf + 20, sizeof(float));
      memcpy(&qw, buf + 24, sizeof(float));
      memcpy(&qx, buf + 28, sizeof(float));
      memcpy(&qy, buf + 32, sizeof(float));
      memcpy(&qz, buf + 36, sizeof(float));

      double norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
      if (norm != 0)
      {
        qw /= norm;
        qx /= norm;
        qy /= norm;
        qz /= norm;
      }
      new_data = true;
      return true;
    }
  }
  return false;
}

/* BMA255 */

class Sensor_BMA255 : public Sensor_IMU
{
public:
  Sensor_BMA255(const uint16_t sen_len, const SensorType sensor_type);
  static SensorBase* Create(const uint16_t sen_len, const SensorType sensor_type)
  {
    return new Sensor_BMA255(sen_len, sensor_type);
  }

private:
  bool parse();
};

Sensor_BMA255::Sensor_BMA255(const uint16_t sen_len, const SensorType sensor_type) : Sensor_IMU(sen_len, sensor_type)
{
  qx = qy = qz = 0;
  qw = 1.0;
  mx = my = mz = 0;
  gx = gy = gz = 0;
}

bool Sensor_BMA255::parse()
{
  if (len >= 6)
  {
    uint8_t* buf = (uint8_t*)get_data();
    if (buf)
    {
      // buffers are a sequences of signed 16bits integers in little-endian
      // order is ax, ay, az
      signed short tmp = LITTLEENDIAN_TO_SIGNED_INT16(buf);
      ax = (float)tmp;
      tmp = LITTLEENDIAN_TO_SIGNED_INT16(buf + 2);
      ay = (float)tmp;
      tmp = LITTLEENDIAN_TO_SIGNED_INT16(buf + 4);
      az = (float)tmp;
      new_data = true;
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
  Sensor_Baro(const uint16_t sen_len, const SensorType sensor_type);
  void publish();
#ifdef HAVE_ROS
  void init_ros(ros::NodeHandle& nh);
#endif

protected:
  float pressure;

private:
  virtual bool parse() = 0;
#ifdef HAVE_ROS
  sensor_msgs::FluidPressure msg;
  ros::Publisher pub;
#endif
};

// IMPL
Sensor_Baro::Sensor_Baro(const uint16_t sen_len, const SensorType sensor_type) : SensorBase(sen_len, sensor_type)
{
}

#ifdef HAVE_ROS
void Sensor_Baro::init_ros(ros::NodeHandle& nh)
{
  pub = nh.advertise<sensor_msgs::FluidPressure>(sensor.name, 10);
  std::cout << "advertized a ros node for a Baro sensor " << sensor.name << std::endl;
}
#endif

void Sensor_Baro::publish()
{
  if (previous_timestamp != timestamp && new_data)
  {
    new_data = false;
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
  Sensor_MPL115A2(const uint16_t sen_len, const SensorType sensor_type);
  static SensorBase* Create(const uint16_t sen_len, const SensorType sensor_type)
  {
    return new Sensor_MPL115A2(sen_len, sensor_type);
  }

private:
  bool parse();
};

Sensor_MPL115A2::Sensor_MPL115A2(const uint16_t sen_len, const SensorType sensor_type)
  : Sensor_Baro(sen_len, sensor_type)
{
}

bool Sensor_MPL115A2::parse()
{
  if (len >= 4)
  {
    uint8_t* buf = (uint8_t*)get_data();
    if (buf)
    {
      memcpy(&pressure, buf, sizeof(float));
      new_data = true;
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
  Sensor_Joy(const uint16_t sen_len, const SensorType sensor_type);
  void publish();
#ifdef HAVE_ROS
  void init_ros(ros::NodeHandle& nh);
#endif

protected:
  std::vector<float> joy_pos;
  std::vector<int> joy_buttons;

private:
  virtual bool parse() = 0;
#ifdef HAVE_ROS
  sensor_msgs::Joy msg;
  ros::Publisher pub;
#endif
};

// IMPL
Sensor_Joy::Sensor_Joy(const uint16_t sen_len, const SensorType sensor_type) : SensorBase(sen_len, sensor_type)
{
}

#ifdef HAVE_ROS
void Sensor_Joy::init_ros(ros::NodeHandle& nh)
{
  pub = nh.advertise<sensor_msgs::Joy>(sensor.name, 10);
  std::cout << "advertized a ros node for a Joy sensor " << sensor.name << std::endl;
}
#endif

void Sensor_Joy::publish()
{
  if (previous_timestamp != timestamp && new_data)
  {
    new_data = false;
    previous_timestamp = timestamp;
#ifdef HAVE_ROS
    msg.header.stamp = ros::Time::now();
    msg.axes = joy_pos;
    msg.buttons = joy_buttons;
    pub.publish(msg);
#else
    // printf something else there
    std::cout << "  timestamp: " << timestamp << "\n\taxes: ";
    for (size_t i = 0; i < joy_pos.size(); i++)
    {
      std::cout << joy_pos[i] << " | ";
    }
    std::cout << std::endl;
    std::cout << "\tbuttons: ";
    for (size_t i = 0; i < joy_buttons.size(); i++)
    {
      std::cout << joy_buttons[i] << " | ";
    }
    std::cout << std::endl;
#endif
  }
}

/* AS5013 */

class Sensor_AS5013 : public Sensor_Joy
{
public:
  Sensor_AS5013(const uint16_t sen_len, const SensorType sensor_type);
  static SensorBase* Create(const uint16_t sen_len, const SensorType sensor_type)
  {
    return new Sensor_AS5013(sen_len, sensor_type);
  }

private:
  bool parse();
};

Sensor_AS5013::Sensor_AS5013(const uint16_t sen_len, const SensorType sensor_type) : Sensor_Joy(sen_len, sensor_type)
{
  joy_pos.resize(2);
  joy_buttons.clear();
}

bool Sensor_AS5013::parse()
{
  if (len >= 2)
  {
    uint8_t* buf = (uint8_t*)get_data();
    if (buf)
    {
      // buffers are a sequences of signed 8bits integers in little-endian
      // order is x, y
      signed char tmp = TO_SIGNED_INT8(buf);
      joy_pos[0] = (float)tmp;
      tmp = TO_SIGNED_INT8(buf + 1);
      joy_pos[1] = (float)tmp;
      new_data = true;
      return true;
    }
  }
  return false;
}

/* AS5013 */

class Sensor_AS5013y : public Sensor_Joy
{
public:
  Sensor_AS5013y(const uint16_t sen_len, const SensorType sensor_type);
  static SensorBase* Create(const uint16_t sen_len, const SensorType sensor_type)
  {
    return new Sensor_AS5013y(sen_len, sensor_type);
  }

private:
  bool parse();
};

Sensor_AS5013y::Sensor_AS5013y(const uint16_t sen_len, const SensorType sensor_type) : Sensor_Joy(sen_len, sensor_type)
{
  joy_pos.resize(2);
  joy_pos[0] = 0;
  joy_buttons.clear();
}

bool Sensor_AS5013y::parse()
{
  if (len >= 1)
  {
    uint8_t* buf = (uint8_t*)get_data();
    if (buf)
    {
      // buffers are a sequences of signed 8bits integers in little-endian
      // order is y
      signed char tmp = TO_SIGNED_INT8(buf);
      joy_pos[1] = (float)tmp;
      new_data = true;
      return true;
    }
  }
  return false;
}

/* Base Myrmex */

// DECL
class Sensor_Tactile : public SensorBase
{
public:
  Sensor_Tactile(const uint16_t sen_len, const SensorType sensor_type);
  void publish();
#ifdef HAVE_ROS
  void init_ros(ros::NodeHandle& nh);
#endif

protected:
  std::vector<float> tactile_array;
#ifdef HAVE_ROS
  ros::Publisher pub;
  sensor_msgs::ChannelFloat32 tactile_sensor;
  tactile_msgs::TactileState msg;
#endif

private:
  virtual bool parse() = 0;
};

// IMPL
Sensor_Tactile::Sensor_Tactile(const uint16_t sen_len, const SensorType sensor_type) : SensorBase(sen_len, sensor_type)
{
}

#ifdef HAVE_ROS
void Sensor_Tactile::init_ros(ros::NodeHandle& nh)
{
  msg.sensors.resize(1);
  pub = nh.advertise<tactile_msgs::TactileState>(sensor.name, 10);
  tactile_sensor.name = "tactile";
  std::cout << "advertized a ros node for a Tactile sensor " << sensor.name << std::endl;
}
#endif

void Sensor_Tactile::publish()
{
  if (previous_timestamp != timestamp && new_data)
  {
    new_data = true;
    previous_timestamp = timestamp;
#ifdef HAVE_ROS
    msg.header.stamp = ros::Time::now();
    tactile_sensor.values = tactile_array;
    msg.sensors[0] = tactile_sensor;
    pub.publish(msg);
#else
    // printf something else there
    std::cout << "  timestamp: " << timestamp << "\n\tdata: ";
    for (size_t i = 0; i < tactile_array.size(); i++)
    {
      std::cout << tactile_array[i] << " | ";
    }
    std::cout << std::endl;
#endif
  }
}

/* MID_tactile_fingertip_teensy */

class Sensor_MID_tactile_fingertip_teensy : public Sensor_Tactile
{
public:
  Sensor_MID_tactile_fingertip_teensy(const uint16_t sen_len, const SensorType sensor_type);
  static SensorBase* Create(const uint16_t sen_len, const SensorType sensor_type)
  {
    return new Sensor_MID_tactile_fingertip_teensy(sen_len, sensor_type);
  }

private:
  bool parse();

private:
  unsigned int num_taxels;
};

Sensor_MID_tactile_fingertip_teensy::Sensor_MID_tactile_fingertip_teensy(const uint16_t sen_len,
                                                                         const SensorType sensor_type)
  : Sensor_Tactile(sen_len, sensor_type)
{
  // sen_len = x * (2 data) => x is the tactile_array size
  num_taxels = sen_len / 2;
  tactile_array.resize(num_taxels);
#ifdef HAVE_ROS
  tactile_sensor.name = "rh_ffdistal";  // matches urdf marker description
#endif
}

bool Sensor_MID_tactile_fingertip_teensy::parse()
{
  if (len >= 1)
  {
    uint8_t* buf = (uint8_t*)get_data();
    if (buf)
    {
      // buffers are a sequences of uint 16 for the data in little-endian
      for (size_t i = 0; i < num_taxels; i++)
      {
        uint16_t tmp = LITTLEENDIANUINT_TO_UNSIGNED_INT16(buf + 2 * i);
        // TODO calibrate here ?
        tactile_array[i] = (float)tmp;
      }
      new_data = true;
      return true;
    }
  }
  return false;
}

/* iObject+ */

class Sensor_iobject_myrmex : public Sensor_Tactile
{
public:
  Sensor_iobject_myrmex(const uint16_t sen_len, const SensorType sensor_type);
  static SensorBase* Create(const uint16_t sen_len, const SensorType sensor_type)
  {
    return new Sensor_iobject_myrmex(sen_len, sensor_type);
  }
  void publish();  // overloaded to permit re-organize the low-level data before
                   // publishing
private:
  bool parse();
#ifdef HAVE_ROS
  void init_ros(ros::NodeHandle& nh);
#endif

private:
  const unsigned int NUM_CHANNELS = 16;
  const unsigned int NUM_BOARDS = 10;
  const unsigned int ADC_PER_BOARD = 6;
  bool initialized;

#ifdef HAVE_ROS
  std::vector<sensor_msgs::ChannelFloat32> tactile_sensors;
  tactile_msgs::TactileState tactile_msg;
#endif
};

Sensor_iobject_myrmex::Sensor_iobject_myrmex(const uint16_t sen_len, const SensorType sensor_type)
  : Sensor_Tactile(sen_len, sensor_type)
{
  // sen_len = 120 => 2*60 bytes => 60 ADC of 16 channels each
  tactile_array.resize(sen_len / 2 * NUM_CHANNELS);
  initialized = false;
}

#ifdef HAVE_ROS
void Sensor_iobject_myrmex::init_ros(ros::NodeHandle& nh)
{
  Sensor_Tactile::init_ros(nh);
  tactile_sensors.resize(NUM_BOARDS);  // 10 boards
  for (size_t board_id = 0; board_id < NUM_BOARDS; board_id++)
  {
    tactile_sensors[board_id].name = "board" + std::to_string(board_id);
    tactile_sensors[board_id].values.resize(NUM_CHANNELS * ADC_PER_BOARD);
  }
}
#endif

// publish only if all tactile data were updated (use channel 0 as a ref)
void Sensor_iobject_myrmex::publish()
{
  if (new_data)
  {
    new_data = false;
#ifdef HAVE_ROS
    // reorganize the data into 10 TactileSensors
    for (size_t board_id = 0; board_id < NUM_BOARDS; board_id++)
    {
      unsigned int board_start_idx = board_id * ADC_PER_BOARD * NUM_CHANNELS;
      for (size_t adc_id = 0; adc_id < ADC_PER_BOARD; adc_id++)
      {
        unsigned int adc_start_idx = adc_id * NUM_CHANNELS;
        for (size_t channel_id = 0; channel_id < NUM_CHANNELS; channel_id++)
        {
          tactile_sensors[board_id].values[adc_id * NUM_CHANNELS + channel_id] =
              tactile_array[board_start_idx + adc_start_idx + channel_id];
        }
      }
    }
    msg.header.stamp = ros::Time::now();
    msg.sensors = tactile_sensors;
    pub.publish(msg);
#else
    // printf something else there
    std::cout << "  timestamp: " << timestamp << "\n\tdata: ";
    for (size_t i = 0; i < tactile_array.size(); i++)
    {
      std::cout << tactile_array[i] << " | ";
    }
    std::cout << std::endl;
#endif
  }
}

bool Sensor_iobject_myrmex::parse()
{
  // TODO:Guillaume handle the timestamp for each channel
  if (len >= 1)
  {
    uint8_t* buf = (uint8_t*)get_data();
    if (buf)
    {
      // buffers are sequences of signed 8bits integers in little-endian
      // order is y

      // buffers are sequences of signed 16bits integers in little-endian
      // there are 30 (block A) + 30 (block B) packets of 2 bytes = 120 bytes = sen_len,
      // 4 MSB are the channel number of the ADC, 12 LSB are data, but interlieved between block A and block B of
      // tactile sensors.
      // here block A will be the 60 first sensors in the array, block B will be the 60 next.
      for (size_t i = 0; i < len / 4; i++)  // 0 - 30
      {
        // BLOCK A
        uint8_t channel = LITTLEENDIAN4MSB_TO_UNSIGNED_INT8(buf + 4 * i);
        // trigger the new_data when the last channel was received
        if (channel == NUM_CHANNELS - 1 && !new_data)
        {
          new_data = true;
          // only validate new_data when the data at zero was also received (so at next cycle)
          if (!initialized)
          {
            initialized = true;
            new_data = false;
          }
        }

        uint16_t tmp = LITTLEENDIAN12_TO_UNSIGNED_INT16(buf + 4 * i);
        size_t idx = i * NUM_CHANNELS + channel;
        if (idx < tactile_array.size())
        {
          // TODO calibrate here ?
          tactile_array[idx] = (float)tmp;
          /*if (1)//tactile_array[idx] < 150.0)
          {
            std::cout << "A c " << std::dec << static_cast<int>(channel)  << ": idx:" << idx << " raw val " << tmp << "
          float val " << tactile_array[idx] << " i: " << i << " hex ";
            std::cout << std::hex <<  static_cast<int>(buf[4 * i]) ;
             std::cout << "|" << std::hex  << static_cast<int>(buf[4 * i + 1]) << " ";
             std::cout << std::hex  << static_cast<int>(buf[4 * i+2]) ;
             std::cout << "|" << std::hex << static_cast<int>(buf[4 * i + 3]) << std::endl;
          }*/
        }
        // BLOCK B
        channel = LITTLEENDIAN4MSB_TO_UNSIGNED_INT8(buf + 4 * i + 2);
        tmp = LITTLEENDIAN12_TO_UNSIGNED_INT16(buf + 4 * i + 2);
        // is after 30 ADC of BLOCK A * 16 channels
        idx = 30 * NUM_CHANNELS + i * NUM_CHANNELS + channel;
        if (idx < tactile_array.size())
        {
          // TODO calibrate here ?
          tactile_array[idx] = (float)tmp;
        }
      }
      return true;
    }
  }
  return false;
}

/* palm baro array */

class Sensor_BMP388modified_pressure_array : public Sensor_Tactile
{
public:
  Sensor_BMP388modified_pressure_array(const uint16_t sen_len, const SensorType sensor_type);
  static SensorBase* Create(const uint16_t sen_len, const SensorType sensor_type)
  {
    return new Sensor_BMP388modified_pressure_array(sen_len, sensor_type);
  }
  void publish();  // overloaded to permit re-organize the low-level data before publishing
private:
  bool parse();
#ifdef HAVE_ROS
  void init_ros(ros::NodeHandle& nh);
#endif

private:
  const unsigned int NUM_BYTE_PER_CHANNELS = 2;
  bool initialized;

#ifdef HAVE_ROS
  std::vector<sensor_msgs::ChannelFloat32> tactile_sensors;
  tactile_msgs::TactileState tactile_msg;
#endif
};

Sensor_BMP388modified_pressure_array::Sensor_BMP388modified_pressure_array(const uint16_t sen_len,
                                                                           const SensorType sensor_type)
  : Sensor_Tactile(sen_len, sensor_type)
{
  // sen_len = 120 => 2*60 bytes => 60 baro
  tactile_array.resize(sen_len / NUM_BYTE_PER_CHANNELS);
  initialized = false;
}

#ifdef HAVE_ROS
void Sensor_BMP388modified_pressure_array::init_ros(ros::NodeHandle& nh)
{
  Sensor_Tactile::init_ros(nh);
  tactile_sensors.resize(1);
  tactile_sensors[0].name = "palm_baro_array";
  tactile_sensors[0].values.resize(tactile_array.size());
}
#endif

// publish only if all tactile data were updated (use channel 0 as a ref)
void Sensor_BMP388modified_pressure_array::publish()
{
  if (new_data)
  {
    new_data = false;
#ifdef HAVE_ROS
    tactile_sensors[0].values = tactile_array;
    msg.header.stamp = ros::Time::now();
    msg.sensors = tactile_sensors;
    pub.publish(msg);
#else
    // printf something else there
    std::cout << "  timestamp: " << timestamp << "\n\tdata: ";
    for (size_t i = 0; i < tactile_array.size(); i++)
    {
      std::cout << tactile_array[i] << " | ";
    }
    std::cout << std::endl;
#endif
  }
}

bool Sensor_BMP388modified_pressure_array::parse()
{
  // TODO:Guillaume handle the timestamp for each channel
  if (len >= 1)
  {
    uint8_t* buf = (uint8_t*)get_data();
    if (buf)
    {
      // buffers are a sequences of unsigned 16bits integers in little-endian
      for (size_t i = 0; i < len / NUM_BYTE_PER_CHANNELS; i++)
      {
        int16_t tmp = LITTLEENDIANUINT_TO_UNSIGNED_INT16(buf + NUM_BYTE_PER_CHANNELS * i);
        size_t idx = i;
        if (idx < tactile_array.size())
        {
          // TODO calibrate here ?
          tactile_array[idx] = (float)tmp;
        }
        new_data = true;
      }
      return true;
    }
  }
  return false;
}

/* tactile_glove */

class Sensor_tactile_glove_teensy : public Sensor_Tactile
{
public:
  Sensor_tactile_glove_teensy(const uint16_t sen_len, const SensorType sensor_type);
  static SensorBase* Create(const uint16_t sen_len, const SensorType sensor_type)
  {
    return new Sensor_tactile_glove_teensy(sen_len, sensor_type);
  }

private:
  bool parse();
// overload the parent function to change the name of the tactile topic to be the backward compatible name
#ifdef HAVE_ROS
  void init_ros(ros::NodeHandle& nh);
#endif

private:
  unsigned int num_taxels;
};

Sensor_tactile_glove_teensy::Sensor_tactile_glove_teensy(const uint16_t sen_len, const SensorType sensor_type)
  : Sensor_Tactile(sen_len, sensor_type)
{
  // sen_len = x * (1 id + 2 data) => x is the tactile_array size
  num_taxels = sen_len / 3;
  tactile_array.resize(num_taxels);
#ifdef HAVE_ROS
  tactile_sensor.name = "tactile glove";  // matches urdf marker description
#endif
}

#ifdef HAVE_ROS
void Sensor_tactile_glove_teensy::init_ros(ros::NodeHandle& nh)
{
  msg.sensors.resize(1);
  pub = nh.advertise<tactile_msgs::TactileState>("TactileGlove", 10);
  tactile_sensor.name = "tactile";
  std::cout << "advertized a ros node for a Tactile sensor " << sensor.name << " on topic TactileGlove" << std::endl;
}
#endif

bool Sensor_tactile_glove_teensy::parse()
{
  if (len >= 1)
  {
    uint8_t* buf = (uint8_t*)get_data();
    if (buf)
    {
      // buffers are a sequences of a uint 8 for the ID and unsigned 16bits integers for the data in little-endian
      for (size_t i = 0; i < num_taxels; i++)
      {
        // id
        size_t idx = TO_UNSIGNED_INT8(buf + 3 * i);
        unsigned short tmp = LITTLEENDIAN12_TO_UNSIGNED_INT16(buf + 3 * i + 1);
        /* unsigned short channel = LITTLEENDIAN4MSB_TO_UNSIGNED_INT8(buf + 3 * i + 1);  // unused for now
        // std::cout << idx << "|" << channel << " ";
        */

        if (idx < tactile_array.size())
        {
          // TODO calibrate here ?
          tactile_array[idx] = 4095.0 - (float)tmp;
        }
      }
      // std::cout << std::endl;
      new_data = true;
      return true;
    }
  }
  return false;
}

/* Base JointState */

// DECL
class Sensor_JointState : public SensorBase
{
public:
  Sensor_JointState(const uint16_t sen_len, const SensorType sensor_type);
  ~Sensor_JointState();
  void publish();
#ifdef HAVE_ROS
  void init_ros(ros::NodeHandle& nh);
#endif

protected:
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> efforts;
#ifdef HAVE_ROS
  ros::Publisher pub;
  sensor_msgs::JointState msg;
#endif
  static size_t nb_sensors;

private:
  virtual bool parse() = 0;
};

//  initialize the nb_sensors, this allows multiple instance of the same sensor to publish to ROS
size_t Sensor_JointState::nb_sensors = 0;

// IMPL
Sensor_JointState::Sensor_JointState(const uint16_t sen_len, const SensorType sensor_type)
  : SensorBase(sen_len, sensor_type)
{
  nb_sensors++;
}

Sensor_JointState::~Sensor_JointState()
{
  Sensor_JointState::nb_sensors--;
}

#ifdef HAVE_ROS
void Sensor_JointState::init_ros(ros::NodeHandle& nh)
{
  std::string suffix = "";
  if (nb_sensors > 1)
    std::string suffix = "_" + std::to_string(nb_sensors - 1);
  pub = nh.advertise<sensor_msgs::JointState>("joint_states" + suffix, 10);
  std::cout << "advertized a ros node for a JointState sensor " << std::endl;
}
#endif

void Sensor_JointState::publish()
{
  if (previous_timestamp != timestamp && new_data)
  {
    new_data = true;
    previous_timestamp = timestamp;
#ifdef HAVE_ROS
    msg.header.stamp = ros::Time::now();
    msg.position = positions;  // should be radian here
    msg.velocity = velocities;
    msg.effort = efforts;
    pub.publish(msg);
#else
    // printf something else there
    std::cout << "  timestamp: " << timestamp << "\n\tposition: ";
    for (size_t i = 0; i < positions.size(); i++)
    {
      std::cout << positions[i] << " | ";
    }
    std::cout << "\n\tvelocities: ";
    for (size_t i = 0; i < velocities.size(); i++)
    {
      std::cout << velocities[i] << " | ";
    }
    std::cout << "\n\tefforts: ";
    for (size_t i = 0; i < efforts.size(); i++)
    {
      std::cout << efforts[i] << " | ";
    }
    std::cout << std::endl;
#endif
  }
}

/* tactile_bend */

class Sensor_tactile_glove_teensy_bend : public Sensor_JointState
{
public:
  Sensor_tactile_glove_teensy_bend(const uint16_t sen_len, const SensorType sensor_type);
  static SensorBase* Create(const uint16_t sen_len, const SensorType sensor_type)
  {
    return new Sensor_tactile_glove_teensy_bend(sen_len, sensor_type);
  }

private:
  bool parse();

private:
  unsigned int num_joints;
};

Sensor_tactile_glove_teensy_bend::Sensor_tactile_glove_teensy_bend(const uint16_t sen_len, const SensorType sensor_type)
  : Sensor_JointState(sen_len, sensor_type)
{
  // sen_len = x * (1 id + 2 data) => x is the joint number
  num_joints = sen_len / 3;
  positions.resize(num_joints);
  velocities.resize(num_joints);
  efforts.resize(num_joints);
#ifdef HAVE_ROS
  for (size_t i = 0; i < num_joints; i++)
  {
    msg.name.push_back("bend_" + std::to_string(i + 1));
  }
#endif
}

bool Sensor_tactile_glove_teensy_bend::parse()
{
  if (len >= 1)
  {
    uint8_t* buf = (uint8_t*)get_data();
    if (buf)
    {
      // buffers are a sequences of a uint 8 for the ID and unsigned 16bits integers for the data in little-endian
      for (size_t i = 0; i < num_joints; i++)
      {
        // id
        size_t idx = TO_UNSIGNED_INT8(buf + 3 * i);
        unsigned short tmp = LITTLEENDIAN12_TO_UNSIGNED_INT16(buf + 3 * i + 1);
        if (idx < positions.size())
        {
          // TODO calibrate here ?
          positions[idx] = 4095.0 - (double)tmp;
        }
      }
      new_data = true;
      return true;
    }
  }
  return false;
}

/* Generic JointState from floats */

class Sensor_generic_position_float : public Sensor_JointState
{
public:
  Sensor_generic_position_float(const uint16_t sen_len, const SensorType sensor_type);
  static SensorBase* Create(const uint16_t sen_len, const SensorType sensor_type)
  {
    return new Sensor_generic_position_float(sen_len, sensor_type);
  }

private:
  bool parse();

private:
  unsigned int num_joints;
};

Sensor_generic_position_float::Sensor_generic_position_float(const uint16_t sen_len, const SensorType sensor_type)
  : Sensor_JointState(sen_len, sensor_type)
{
  // sen_len = x * (4 B of float) => x is the joint number
  num_joints = sen_len / 4;
  positions.resize(num_joints);
  velocities.resize(num_joints);
  efforts.resize(num_joints);
#ifdef HAVE_ROS
  for (size_t i = 0; i < num_joints; i++)
  {
    msg.name.push_back("pos_" + std::to_string(i + 1));
  }
#endif
}

bool Sensor_generic_position_float::parse()
{
  if (len >= 1)
  {
    uint8_t* buf = (uint8_t*)get_data();
    if (buf)
    {
      // buffers are a sequences of a uint 8 for the ID and unsigned 16bits integers for the data in little-endian
      for (size_t i = 0; i < num_joints; i++)
      {
        float tmp = *((float*)buf + i);
        positions[i] = (double)tmp;
      }
      new_data = true;
      return true;
    }
  }
  return false;
}
}
#endif
