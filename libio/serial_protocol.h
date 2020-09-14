#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H
#include <string>
#include <vector>
#include <map>
#include <utility>
#include <stdint.h>
#include "serial_com.h"
#include "serial_protocol_defines.h"
#ifdef HAVE_ROS
#include <ros/ros.h>
#include <agni_serial_protocol/SetPeriod.h>
#endif

// constants
#define SP_MAX_BUF_SIZE 256

// small shift operations to combine/split bytes
#define Highbyte(x) (x >> 8)
#define Lowbyte(x) (x & 0xff)

namespace serial_protocol
{
struct DeviceType
{
  uint8_t id;
  std::string name;
  std::string description;
};

struct SensorType
{
  uint16_t id;
  std::string name;
  std::string manufacturer;
  std::string description;
  std::string parser_library;
  uint16_t data_length;
};

class SensorBase
{
public:
  explicit SensorBase(const uint16_t sen_len, const SensorType sensor_type);
  virtual ~SensorBase();
  bool init();
#ifdef HAVE_ROS
  virtual void init_ros(ros::NodeHandle& nh) = 0;
#endif
  bool unpack(uint8_t* buf);
  void* get_data();
  uint32_t get_timestamp();
  uint16_t get_len()
  {
    return len;
  }
  virtual void publish() = 0;
  virtual bool parse() = 0;

protected:
  void extract_timestamp(uint8_t* buf);
  SensorType sensor;
  uint16_t len;
  void* dataptr;
  uint32_t timestamp;
  uint32_t previous_timestamp;
  bool new_data;
  uint8_t base_sensor_id;

private:
  static uint8_t base_sensor_count;
};

typedef SensorBase* (*CreateSensorFn)(const uint16_t sen_len, const SensorType sensor_type);

/* **********************
 *
 * based on Factory Pattern in C++
 * Cale Dunlap, 15 Sep 2012
 *
 */

class SensorFactory
{
private:
  SensorFactory(const SensorFactory&);
  SensorFactory();
  SensorFactory& operator=(const SensorFactory&)
  {
    return *this;
  }

  typedef std::map<std::string, CreateSensorFn> SensorFactoryMap;
  SensorFactoryMap factory_map;

public:
  ~SensorFactory()
  {
    factory_map.clear();
  }

  static SensorFactory* Get()
  {
    static SensorFactory instance;
    return &instance;
  }

  void Register(const std::string& sensor_name, CreateSensorFn fnCreate);
  SensorBase* CreateSensor(const uint16_t sen_len, const SensorType sensor_type);
};

class Device
{
public:
  Device();
  ~Device();
  void init(const DeviceType dev_type);
#ifdef HAVE_ROS
  void init_ros(ros::NodeHandle& nh);
#endif
  std::string get_serial();

  std::vector<std::pair<SensorBase*, bool>>& get_sensors()
  {
    return sensors;
  }
  std::pair<SensorBase*, bool>* get_sensor_by_idx(const uint8_t idx);
  bool exists_sensor(const uint8_t idx);
  void add_sensor(const uint16_t data_len, const SensorType sensor_type);
  void publish_all();

  DeviceType device;

protected:
  std::vector<std::pair<SensorBase*, bool>> sensors;
  // std::vector<bool> active_sensors;
  uint32_t cached_max_stream_size;
  bool active_sensor_modified;
  std::string serialnum;
};

class SerialProtocolBase
{
public:
  explicit SerialProtocolBase(SerialCom* serial_com, const std::string device_filename = "",
                              const std::string sensor_filename = "");
  ~SerialProtocolBase();
  bool init();
#ifdef HAVE_ROS
  void init_ros(ros::NodeHandle& nh);
#endif

  bool set_device(const uint8_t dev_id);
  void start_streaming(const uint8_t mode = SP_CMD_START_STREAM_CONT_ALL);
  void update();
  void publish();
  void set_period(const uint8_t sen_id, const uint16_t period);
  void set_periods(const std::map<uint8_t, uint16_t> period_map);
  // void process();
  bool get_data_as_float(float& val, uint8_t did);
  bool get_data_as_short(short& val, uint8_t did);
  bool get_data_as_unsigned_short(unsigned short& val, const uint8_t did);
  bool get_data_as_3_float(float& x, float& y, float& z, const uint8_t did);

  uint32_t get_timestamp(const uint8_t did);
  void trigger(const uint8_t mode, const uint8_t sen_id);
  void stop_streaming();

  void read_device_types(const uint8_t v);
  void read_sensor_types(const uint8_t v);
  bool exists_device(const uint8_t dev_id);
  bool exists_sensor_driver(const uint16_t sen_driver_id);
  bool exists_sensor(const uint8_t sen_id);

  DeviceType get_device();
  bool verbose;

protected:
  void config();
  void read_config(uint8_t* buf);
  void read_error(uint8_t* buf);
  void read_data(uint8_t* buf, const uint8_t did);
  void read();

  // void unpack_data(uint8_t *buf);
  bool valid_data(const uint8_t* buf, const uint32_t buf_len);
  bool init_device_from_config(uint8_t* buf, const uint8_t config_num);

  bool valid_header(const uint8_t* buf);
  bool valid_checksum(const uint8_t* buf, const uint32_t len);
  uint8_t compute_checksum(const uint8_t* buf, const uint32_t len);

  void send(const uint8_t* buf, const uint32_t len);
  uint32_t gen_command(uint8_t* buf, const uint8_t destination, const uint8_t command,
                       const uint32_t size, const uint16_t stride=1, const uint8_t* data = NULL);
  uint32_t gen_master_ping_req(uint8_t* buf);
  uint32_t gen_master_config_req(uint8_t* buf);
  uint32_t gen_sensor_trigger_req(uint8_t* buf, uint8_t sen_id);
  uint32_t gen_master_trigger_req(uint8_t* buf);
  uint32_t gen_topo_req(uint8_t* buf);
  uint32_t gen_serialnum_req(uint8_t* buf);
  uint32_t gen_period_master_req(uint8_t* buf, const std::map<uint8_t, uint16_t>& period_map);
  uint32_t gen_period_sensor_req(uint8_t* buf, const uint8_t sen_id, const uint16_t period);

  uint8_t version;
  bool streaming;
  std::map<uint8_t, DeviceType> device_types;
  std::map<uint16_t, SensorType> sensor_types;
  SerialCom* s;
  std::string d_filename;
  std::string s_filename;
  Device dev;

#ifdef HAVE_ROS
  ros::ServiceServer service_set_period;
  bool service_set_period_cb(agni_serial_protocol::SetPeriod::Request& req,
                                               agni_serial_protocol::SetPeriod::Response& res);
#endif
};
}
#endif
