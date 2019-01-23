#ifndef REG_SENSORS_IMPL_H
#define REG_SENSORS_IMPL_H
#include "serial_protocol.h"
#include <cstring>
#include <string>
#include <stdint.h>

namespace serial_protocol {

// DECL
class Sensor_Default : public SensorBase
{
public:

  Sensor_Default(const unsigned int sen_len, const SensorType sensor_type);
  ~Sensor_Default();
  bool init();
  bool parse(uint8_t *buf);

  static SensorBase* Create(const unsigned int sen_len, const SensorType sensor_type) { return new Sensor_Default(sen_len, sensor_type); }
};

// IMPL
Sensor_Default::Sensor_Default(const unsigned int sen_len, const SensorType sensor_type) : SensorBase(sen_len, sensor_type) {}

bool Sensor_Default::init()
{
  dataptr = (void*) new uint8_t[len];
  if (dataptr == NULL)
    return false;
  return true;
}

Sensor_Default::~Sensor_Default()
{
  if (dataptr)
    delete (uint8_t*)dataptr;
//  delete this;
  //SensorBase::~SensorBase();
}

bool Sensor_Default::parse(uint8_t *buf)
{
  // store the raw value, no processing at all
  if (!dataptr)
    return false;
  extract_timestamp(buf);
  std::memcpy((uint8_t *)dataptr, buf + TIMESTAMP_LEN, len);
  return true;
}

// DECL
class Sensor_IMU_MPU9250 : public SensorBase
{
public:

  Sensor_IMU_MPU9250(const unsigned int sen_len, const SensorType sensor_type);
  ~Sensor_IMU_MPU9250();
  bool init();
  bool parse(uint8_t *buf);

  static SensorBase* Create(const unsigned int sen_len, const SensorType sensor_type) { return new Sensor_IMU_MPU9250(sen_len, sensor_type); }
};

// IMPL
Sensor_IMU_MPU9250::Sensor_IMU_MPU9250(const unsigned int sen_len, const SensorType sensor_type) : SensorBase(sen_len, sensor_type) {}

bool Sensor_IMU_MPU9250::init()
{
  dataptr = (void*) new uint8_t[len];
  if (dataptr == NULL)
    return false;
  return true;
}

Sensor_IMU_MPU9250::~Sensor_IMU_MPU9250()
{
  if (dataptr)
    delete (uint8_t*)dataptr;
  //delete this;
  //SensorBase::~SensorBase();
}

bool Sensor_IMU_MPU9250::parse(uint8_t *buf)
{
  // store the raw value, no processing at all
  if (!dataptr)
    return false;
  extract_timestamp(buf);
  std::memcpy((uint8_t *)dataptr, buf + TIMESTAMP_LEN, len);
  return true;
}


// DECL
class Sensor_IMU_MPU9250_Acc : public SensorBase
{
public:

  Sensor_IMU_MPU9250_Acc(const unsigned int sen_len, const SensorType sensor_type);
  ~Sensor_IMU_MPU9250_Acc();
  bool init();
  bool parse(uint8_t *buf);

  static SensorBase* Create(const unsigned int sen_len, const SensorType sensor_type) { return new Sensor_IMU_MPU9250_Acc(sen_len, sensor_type); }
};

// IMPL
Sensor_IMU_MPU9250_Acc::Sensor_IMU_MPU9250_Acc(const unsigned int sen_len, const SensorType sensor_type) : SensorBase(sen_len, sensor_type) {}

bool Sensor_IMU_MPU9250_Acc::init()
{
  dataptr = (void*) new uint8_t[len];
  if (dataptr == NULL)
    return false;
  return true;
}

Sensor_IMU_MPU9250_Acc::~Sensor_IMU_MPU9250_Acc()
{
  if (dataptr)
    delete (uint8_t*)dataptr;
  //delete this;
  //SensorBase::~SensorBase();
}

bool Sensor_IMU_MPU9250_Acc::parse(uint8_t *buf)
{
  // store the raw value, no processing at all
  if (!dataptr)
    return false;
  extract_timestamp(buf);
  std::memcpy((uint8_t *)dataptr, buf + TIMESTAMP_LEN, len);
  return true;
}


}
#endif
