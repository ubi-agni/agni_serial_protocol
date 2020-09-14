#include "serial_protocol.h"
#include "registered_sensors_impl.h"
#include "yaml-cpp/yaml.h"
#include <exception>
#include <stdexcept>
#include <cstring>
#include <string>
#include <sstream>
#include <iostream>
#include <utility>

namespace serial_protocol
{
uint8_t SensorBase::base_sensor_count{ 0 };

SensorBase::SensorBase(const uint16_t sen_len, const SensorType sensor_type)
  : sensor(sensor_type), len(0), dataptr(0), timestamp(0), previous_timestamp(0)
{
  // check data len consistency to registered len
  if (sensor_type.data_length > 0)
  {
    if (sen_len != sensor_type.data_length)
      throw std::runtime_error(std::string("sensor: non-matching data length"));
  }
  len = sen_len;
  base_sensor_id = ++base_sensor_count;
}

SensorBase::~SensorBase()
{
  if (dataptr)
    delete (uint8_t*)dataptr;
  base_sensor_count--;
}

bool SensorBase::init()
{
  if (len)
  {
    if (dataptr != 0)
      delete (uint8_t*)dataptr;
    dataptr = (void*)new uint8_t[len];
    if (dataptr != 0)
      return true;
  }
  return false;
}

bool SensorBase::unpack(uint8_t* buf)
{
  // store the raw value, no processing at all
  if (!dataptr)
    return false;
  extract_timestamp(buf);
  std::memcpy((uint8_t*)dataptr, buf + SP_TIMESTAMP_LEN, len);
  return true;
}

void* SensorBase::get_data()
{
  return dataptr;
}

uint32_t SensorBase::get_timestamp()
{
  return timestamp;
}

void SensorBase::extract_timestamp(uint8_t* buf)
{
  std::memcpy((uint8_t*)&timestamp, buf, SP_TIMESTAMP_LEN);
}

/* **********************
 *
 * based on Factory Pattern in C++
 * Cale Dunlap, 15 Sep 2012
 *
 */

SensorFactory::SensorFactory()
{
  Register("undefined", &Sensor_Default::Create);
  Register("MID_tactile_fingertip_teensy", &Sensor_MID_tactile_fingertip_teensy::Create);
  Register("BNO055", &Sensor_BNO055::Create);
  Register("BNO08X", &Sensor_BNO08X::Create);
  Register("BMA255", &Sensor_BMA255::Create);
  Register("MPU9250", &Sensor_MPU9250::Create);
  Register("MPU9250_accelerometer", &Sensor_IMU_MPU9250_Acc::Create);
  Register("mpl115a2", &Sensor_MPL115A2::Create);
  Register("AS5013_y_position", &Sensor_AS5013y::Create);
  Register("AS5013", &Sensor_AS5013::Create);
  Register("iobject_myrmex", &Sensor_iobject_myrmex::Create);
  Register("tactile_glove_teensy", &Sensor_tactile_glove_teensy::Create);
  Register("tactile_glove_teensy_bend", &Sensor_tactile_glove_teensy_bend::Create);
  Register("BMP388modified_pressure_array", &Sensor_BMP388modified_pressure_array::Create);
  Register("BMP388_testboard_4sensors", &Sensor_BMP388modified_pressure_array::Create);
  Register("Generic_position_sensors_float", &Sensor_generic_position_float::Create);
}

void SensorFactory::Register(const std::string& sensor_name, CreateSensorFn fnCreate)
{
  factory_map[sensor_name] = fnCreate;
}

SensorBase* SensorFactory::CreateSensor(const uint16_t sen_len, const SensorType sensor_type)
{
  SensorFactoryMap::iterator it = factory_map.find(sensor_type.name);
  if (it != factory_map.end())
    return it->second(sen_len, sensor_type);
  return NULL;
}

/* *********************** */

Device::Device()
{
  device.id = 0;
}

void Device::init(const DeviceType dev_type)
{
  device = dev_type;
}

Device::~Device()
{
  std::vector<std::pair<SensorBase*, bool>>::iterator s;
  for (s = sensors.begin(); s != sensors.end(); s++)
  {
    delete (*s).first;
  }
}

void Device::init_ros(ros::NodeHandle& nh)
{
  std::vector<std::pair<SensorBase*, bool>>::iterator s;
  for (s = sensors.begin(); s != sensors.end(); s++)
  {
    SensorBase* sensor = (*s).first;
    sensor->init_ros(nh);
  }
}

std::string Device::get_serial()
{
  return serialnum;
}

std::pair<SensorBase*, bool>* Device::get_sensor_by_idx(const uint8_t idx)
{
  int sensor_idx = idx - 1;
  if (sensor_idx >= 0 && sensor_idx < (int)sensors.size())
    return &sensors[sensor_idx];
  else
    return NULL;
}

bool Device::exists_sensor(const uint8_t idx)
{
  if (idx == 0)
    return 0;
  else
    return idx <= sensors.size();
}

void Device::add_sensor(const uint16_t data_len, const SensorType sensor_type)
{
  try
  {
    SensorBase* sensor = SensorFactory::Get()->CreateSensor(data_len, sensor_type);

    if (sensor)
    {
      if (sensor->init())
      {
        sensors.push_back(std::make_pair(sensor, true));
      }
      else
      {
        delete sensor;
        throw std::runtime_error(std::string("sp: config answer from non-master not supported"));
      }
    }
    else
    {
      std::cerr << "no sensor library for this sensor " << sensor_type.name << ", using default one" << std::endl;
      SensorType st;
      st.id = 0x0000;
      st.name = "undefined";
      st.data_length = data_len;
      sensor = SensorFactory::Get()->CreateSensor(data_len, st);
      if (sensor)
      {
        if (sensor->init())
        {
          sensors.push_back(std::make_pair(sensor, true));
        }
        else
        {
          delete sensor;
          throw std::runtime_error(std::string("sp: config answer from non-master not supported"));
        }
        // throw std::runtime_error(std::string("sp: could not create a sensor"));
      }
      else
      {
        throw std::runtime_error(std::string("sp: could not create a sensor"));
      }
    }
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    throw std::runtime_error(std::string("sp: could not add a sensor"));
  }
}

void Device::publish_all()
{
  std::vector<std::pair<SensorBase*, bool>>::iterator it;
  for (it = sensors.begin(); it != sensors.end(); it++)
  {
    std::pair<SensorBase*, bool> item = *it;
    if (item.second)
    {
      item.first->publish();
    }
  }
}

/* *********************** */

SerialProtocolBase::SerialProtocolBase(SerialCom* serial_com, const std::string device_filename,
                                       const std::string sensor_filename)
  : verbose(false), streaming(false), s(serial_com), d_filename(device_filename), s_filename(sensor_filename)
{
}

SerialProtocolBase::~SerialProtocolBase()
{
}

bool SerialProtocolBase::init()
{
  try
  {
    stop_streaming();
    config();
    return true;
  }
  catch (const std::exception& e)
  {
    std::cerr << "sp: init failed:" << e.what() << std::endl;
    return false;
  }
}

#ifdef HAVE_ROS
void SerialProtocolBase::init_ros(ros::NodeHandle& nh)
{
  dev.init_ros(nh);
  // initialize set_period services
  service_set_period = nh.advertiseService("set_period", &SerialProtocolBase::service_set_period_cb, this) ;
}

bool SerialProtocolBase::service_set_period_cb(agni_serial_protocol::SetPeriod::Request& req,
                                               agni_serial_protocol::SetPeriod::Response& res)
{
  res.result = agni_serial_protocol::SetPeriod::Response::FAILED;
  size_t size = req.period_map.sensor_ids.size();
  if (size)
  {
    if (size == req.period_map.periods.size())  // valid data with same size
    {
      if (size == 1)  // one logical sensor
      {
        unsigned int period = req.period_map.periods[0] * 10.0;
        if (period >= 1 && period < 65536)  // accept value higher than one times 100 us
        {
          try
          {
            set_period(req.period_map.sensor_ids[0], (uint16_t)period);
            res.result = agni_serial_protocol::SetPeriod::Response::SUCCESS;
          }
          catch (const std::exception& e)
          {
            std::stringstream sstr;
            sstr  << "Failed to set period: " << e.what();
            res.message = sstr.str();
          }
        }
        else
        {
          res.message = "period must be >= 0.1 and < 6553.5 ms";
        }
      }
      else
      {
        std::map<uint8_t, uint16_t> period_map;
        unsigned int invalid_period = 0;
        for (unsigned int i = 0; i < req.period_map.sensor_ids.size(); ++i)
        {
          unsigned int period = req.period_map.periods[i] * 10.0;
          uint8_t sen_id = req.period_map.sensor_ids[i];
          if (period >= 1 && period < 65536 && exists_sensor(sen_id))  // accept value higher than one times 100 us
          {
            period_map[req.period_map.sensor_ids[i]] = (uint16_t)period;
          }
          else
          {
            invalid_period++;
          }
        }
        try
        {
          set_periods(period_map);
          
          if (invalid_period)
          {
            std::stringstream sstr;
            sstr << invalid_period << " out of range periods were skipped";
            res.message = sstr.str();
            res.result = agni_serial_protocol::SetPeriod::Response::INCOMPLETE;
          }
          else
            res.result = agni_serial_protocol::SetPeriod::Response::SUCCESS;
        }
        catch (const std::exception& e)
        {
          std::stringstream sstr;
          sstr  << "Failed to set period: " << e.what();
          res.message = sstr.str();
        }
      }
    }
    else
    {
      res.message = "Mismatching sensor_ids/periods size";
    }
  }
  else
  {
    res.message = "Set period cannot be empty";
  }
  return true;
}

#endif

void SerialProtocolBase::config()
{
  uint8_t send_buf[SP_MAX_BUF_SIZE];
  // request config
  uint32_t send_buf_len = gen_master_config_req(send_buf);
  try
  {
    // printf("sp: fake write\n");
    s->writeFrame(send_buf, (size_t)send_buf_len);
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    throw std::runtime_error(std::string("sp: failed to config"));
  }
  read();
}

bool SerialProtocolBase::init_device_from_config(uint8_t* buf, const uint8_t config_num)
{
  for (uint8_t i = 0; i < config_num; i++)
  {
    uint8_t* config_buf = buf + i * SP_SDSC_SIZE * sizeof(uint8_t);
    uint16_t sen_type = config_buf[0] + config_buf[1] * 256;
    uint16_t sen_len = config_buf[2] + config_buf[3] * 256;
    if (verbose)
      printf("sp: for sensor %d, found sen_type %u and sen_len %u\n", i, sen_type, sen_len);
    if (exists_sensor_driver(sen_type))
    {
      try
      {
        dev.add_sensor(sen_len, sensor_types[sen_type]);
      }
      catch (const std::exception& e)
      {
        std::cerr << e.what() << std::endl;
        return false;
      }
    }
    else
      return false;
  }
  return true;
}

bool SerialProtocolBase::set_device(const uint8_t dev_id)
{
  if (exists_device(dev_id))
  {
    dev.init(device_types[dev_id]);
    return true;
  }
  else
    return false;
}

void SerialProtocolBase::set_period(const uint8_t sen_id, const uint16_t period)
{
  uint8_t send_buf[SP_HEADER_LEN + SP_DID_LEN + SP_CMD_LEN + SP_CMD_DATA_SZ_LEN + SP_PERIOD_SENSOR_DATA_LEN + SP_CHKSUM_LEN];
  uint32_t send_buf_len = 0;
  if (sen_id == 0)
  {
    throw std::runtime_error(std::string("sp: set_period: sen_id cannot be 0 for set period"));
  }
  else
  {
    // check if sensor exists
    if (exists_sensor(sen_id))
    {
      send_buf_len = gen_period_sensor_req(send_buf, sen_id, period);
    }
    else
    {
      throw std::runtime_error(std::string("sp: set_period: sen_id does not exist"));
    }
  }
  if (send_buf_len)
  {
    try
    {
      send(send_buf, send_buf_len);
    }
    catch (const std::exception& e)
    {
      std::cerr << e.what() << std::endl;
      throw std::runtime_error(std::string("sp: failed to set period"));
    }
  }
  else
  {
    throw std::runtime_error(std::string("sp: invalid set period request"));
  }
}

void SerialProtocolBase::set_periods(const std::map<uint8_t, uint16_t> period_map)
{
  if (period_map.size())
  {
    uint8_t send_buf[SP_HEADER_LEN + SP_DID_LEN + SP_CMD_LEN + SP_CMD_DATA_SZ_LEN + SP_PERIOD_DATA_LEN * period_map.size() + SP_CHKSUM_LEN];
    uint32_t send_buf_len = 0;
    send_buf_len = gen_period_master_req(send_buf, period_map);
    if (send_buf_len)
    {
      try
      {
        send(send_buf, send_buf_len);
      }
      catch (const std::exception& e)
      {
        std::cerr << e.what() << std::endl;
        throw std::runtime_error(std::string("sp: failed to set periods"));
      }
    }
    else
    {
      throw std::runtime_error(std::string("sp: invalid set periods request"));
    }
  }
}

bool SerialProtocolBase::valid_data(const uint8_t* buf, const uint32_t buf_len)
{
  return valid_header(buf) && valid_checksum(buf, buf_len);
}

bool SerialProtocolBase::valid_header(const uint8_t* buf)
{
  return (uint16_t)(buf[0] * 256 + buf[1]) == SP_HEADER;
}

bool SerialProtocolBase::valid_checksum(const uint8_t* buf, const uint32_t len)
{
  return compute_checksum(buf, len - 1) == buf[len - 1];
}

uint8_t SerialProtocolBase::compute_checksum(const uint8_t* buf, const uint32_t len)
{
  uint8_t xorsum = 0;
  for (uint32_t i = 0; i < len; i++)  // do xor for each data byte
  {
    xorsum = xorsum ^ buf[i];
  }
  return xorsum;
}

void SerialProtocolBase::read_device_types(const uint8_t v)
{
  // read params for given version
  if (d_filename != "")
  {
    if (verbose)
      std::cout << "sp: reading device_types from " << d_filename << std::endl;
    // bool success = false;
    try
    {
      YAML::Node root = YAML::LoadFile(d_filename);
      if (root["agni_serial_protocol"])
      {
        YAML::Node agni_serial_protocol = root["agni_serial_protocol"];
        for (YAML::iterator it = agni_serial_protocol.begin(); it != agni_serial_protocol.end(); ++it)
        {
          // std::string protocol_version_str = it->first.as<std::string>();
          // std::string protocol_version_str =
          uint8_t protocol_version = (uint8_t)it->first.as<int>();
          if (verbose)
            std::cout << "sp: found specs for version " << protocol_version << "\n";
          const YAML::Node& version_node = it->second;
          if (verbose)
            std::cout << "sp: extract version\n";
          if (version_node["device_types"])
          {
            if (v == protocol_version)
            {
              YAML::Node device_types_node = version_node["device_types"];
              if (verbose)
                std::cout << "sp: found specs for device_types \n";
              for (YAML::const_iterator devit = device_types_node.begin(); devit != device_types_node.end(); ++devit)
              {
                YAML::Node device_types_item = *devit;  //->second;
                if (verbose)
                  std::cout << "sp: extract device_type item ";
                DeviceType dt;
                std::stringstream sstr("");
                sstr << std::hex << device_types_item["type"].as<std::string>();
                int did;
                sstr >> did;
                dt.id = (uint8_t)did;
                dt.name = device_types_item["name"].as<std::string>();
                if (verbose)
                  std::cout << (int)dt.id << " named: " << dt.name << "\n";
                dt.description = device_types_item["description"].as<std::string>();
                device_types[dt.id] = dt;
              }
              // success = true;
              return;
            }
            else
            {
              std::cerr << "sp: yaml does not contain specs for version " << (int)v << "\n";
            }
          }
          else
          {
            std::cerr << "sp: yaml does not contain device_types \n";
          }
        }
      }
      else
      {
        std::cerr << "sp: yaml does not contain agni_serial_protocol \n";
      }
    }
    catch (const std::exception& e)
    {
      std::cerr << "sp: read device_types error: " << e.what() << std::endl;
    }
  }
  std::cout << "sp: using default device_types\n";

  // fallback

  DeviceType dt;
  dt.id = 0x00;
  dt.name = "undefined";
  dt.description = "undefined";
  device_types[dt.id] = dt;

  dt.id = 0x01;
  dt.name = "prototype";
  dt.description = "experimental hardware";
  device_types[dt.id] = dt;
}

void SerialProtocolBase::read_sensor_types(const uint8_t v)
{
  // read params for given version
  if (s_filename != "")
  {
    if (verbose)
      std::cout << "sp: reading sensor_types from " << s_filename << std::endl;
    // bool success = false;
    try
    {
      YAML::Node root = YAML::LoadFile(s_filename);
      if (root["agni_serial_protocol"])
      {
        YAML::Node agni_serial_protocol = root["agni_serial_protocol"];
        for (YAML::iterator it = agni_serial_protocol.begin(); it != agni_serial_protocol.end(); ++it)
        {
          uint8_t protocol_version = (uint8_t)it->first.as<int>();
          if (verbose)
            std::cout << "sp: found specs for version " << (int)protocol_version << "\n";
          const YAML::Node& version_node = it->second;
          if (verbose)
            std::cout << "sp: extract version\n";
          if (version_node["registered_devices"])
          {
            if (v == protocol_version)
            {
              YAML::Node sensor_types_node = version_node["registered_devices"];
              if (verbose)
                std::cout << "sp: found specs for registered_devices \n";
              for (YAML::const_iterator senit = sensor_types_node.begin(); senit != sensor_types_node.end(); ++senit)
              {
                YAML::Node sensor_types_item = *senit;  //->second;
                if (verbose)
                  std::cout << "sp: extract sensor_type item ";
                SensorType st;
                std::stringstream sstr("");
                sstr << std::hex << sensor_types_item["uid"].as<std::string>();
                int type_id;
                sstr >> type_id;
                st.id = (uint16_t)type_id;
                st.name = sensor_types_item["name"].as<std::string>();
                st.manufacturer = sensor_types_item["manufacturer"].as<std::string>();
                st.description = sensor_types_item["description"].as<std::string>();
                st.parser_library = sensor_types_item["parser_library"].as<std::string>();
                st.data_length = (uint16_t)sensor_types_item["data_length"].as<int>();
                if (verbose)
                  std::cout << (int)st.id << " named: " << st.name << " with length " << st.data_length << "\n";
                sensor_types[st.id] = st;
              }
              // success = true;
              return;
            }
            else
            {
              std::cerr << "sp: yaml does not contain specs for version " << (int)v << "\n";
            }
          }
          else
          {
            std::cerr << "sp: yaml does not contain sensor_types \n";
          }
        }
      }
      else
      {
        std::cerr << "sp: yaml does not contain agni_serial_protocol \n";
      }
    }
    catch (const std::exception& e)
    {
      std::cerr << "sp: read sensor_types error: " << e.what() << std::endl;
    }
  }
  std::cout << "sp: using default sensor_types\n";

  // hardcoded for now
  SensorType st;
  st.id = 0x0000;
  st.name = "undefined";
  st.manufacturer = "undefined";
  st.description = "undefined";
  st.parser_library = "dummy::dummy";
  st.data_length = 0;
  sensor_types[st.id] = st;

  st.id = 0xDC10;
  st.name = "MPU9250";
  st.manufacturer = "Drotek";
  st.description = "MPU9250 Drotek Inertial Measurement Unit";
  st.parser_library = "imu::mp9250";
  st.data_length = 2;
  sensor_types[st.id] = st;
}

bool SerialProtocolBase::exists_device(const uint8_t dev_id)
{
  return device_types.find(dev_id) != device_types.end();
}

bool SerialProtocolBase::exists_sensor_driver(const uint16_t sen_driver_id)
{
  return sensor_types.find(sen_driver_id) != sensor_types.end();
}

bool SerialProtocolBase::exists_sensor(const uint8_t sen_id)
{
  return dev.exists_sensor(sen_id);
}

DeviceType SerialProtocolBase::get_device()
{
  return dev.device;
}

void SerialProtocolBase::trigger(const uint8_t mode, const uint8_t sen_id)
{
  if (!streaming)
  {
    uint8_t send_buf[5];
    uint32_t send_buf_len = 0;
    if (sen_id == 0)
    {
      send_buf_len = gen_master_trigger_req(send_buf);
    }
    else
    {
      // check if sensor exists
      if (exists_sensor(sen_id))
      {
        send_buf_len = gen_sensor_trigger_req(send_buf, sen_id);
      }
    }
    if (send_buf_len)
    {
      try
      {
        send(send_buf, send_buf_len);
      }
      catch (const std::exception& e)
      {
        std::cerr << e.what() << std::endl;
        throw std::runtime_error(std::string("sp: failed to trigger"));
      }
    }
    else
    {
      throw std::runtime_error(std::string("sp: invalid trigger request"));
    }
  }
  else
  {
    throw std::runtime_error(std::string("sp: cannot trigger while streaming, stop streaming first"));
  }
}

void SerialProtocolBase::update()
{
  try
  {
    read();
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    throw std::runtime_error(std::string());
  }
}

void SerialProtocolBase::publish()
{
  dev.publish_all();
#ifdef HAVE_ROS
  ros::spinOnce();
#endif
}

void SerialProtocolBase::read_config(uint8_t* buf)
{
  // read beginning of config message
  /*size_t buf_len = 0;
  unsigned int retry = 0;
  while (retry < 5 && buf_len==0)
  {
    try
    {
      buf_len = s->readFrame(buf, CONF_MIN_LEN);
    }
    catch (const std::exception &e) {
      std::cerr << " " <<  e.what() << std::endl;
      if (retry ==5)
        throw std::runtime_error(std::string("sp: device failed to answer"));
    }
    retry++;
  }*/
  // read additional config header
  size_t buf_len = 0;
  buf_len = s->readFrame(buf + SP_VERSION_OFFSET, SP_VERSION_LEN + SP_DEVID_LEN + SP_SDSC_SZ_LEN);
  if (buf_len < SP_VERSION_LEN + SP_DEVID_LEN + SP_SDSC_SZ_LEN)
  {
    std::cerr << "sp: incorrect config header size: " << buf_len << " < "
              << SP_VERSION_LEN + SP_DEVID_LEN + SP_SDSC_SZ_LEN << "\n";
    throw std::runtime_error(std::string("sp: device did not answer enough data"));
  }
  // read versions
  uint8_t version = (uint8_t)buf[SP_VERSION_OFFSET];
  if (version > 0 and version <= SP_MAX_KNOWN_VERSION)
  {
    read_device_types(version);
    read_sensor_types(version);
  }
  else
  {
    std::cerr << "sp: found unknown version ("
              << "0x" << std::uppercase << std::hex << version << ")\n";
    throw std::runtime_error(std::string("sp: unsupported version"));
  }

  // validate master is answering
  if (buf[SP_DID_OFFSET] != SP_DID_MASTER)
  {
    throw std::runtime_error(std::string("sp: config answer from non-master not supported"));
  }

  // create a device
  if (!set_device(buf[SP_DEVID_OFFSET]))
  {
    throw std::runtime_error(std::string("sp: device exists already"));
  }

  // get how much more config comes
  uint8_t config_num = (uint8_t)buf[SP_SDSC_OFFSET];
  // read next part of the config message
  size_t config_len = 0;
  try
  {
    config_len = s->readFrame(buf + SP_SDSC_DATA_OFFSET, config_num * SP_SDSC_SIZE + SP_CHKSUM_LEN);
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    throw std::runtime_error(std::string("sp: device failed to answer"));
  }
  // check data
  if (config_len != (size_t)(config_num * SP_SDSC_SIZE + SP_CHKSUM_LEN))
  {
    std::cerr << "sp: read " << config_len << " bytes of config_len instead of "
              << config_num * SP_SDSC_SIZE + SP_CHKSUM_LEN << std::endl;
    throw std::runtime_error(std::string("sp: incomplete sensor description datagram"));
  }
  // validate the full frame
  if (valid_data(buf, SP_SDSC_DATA_OFFSET + config_len))
  {
    if (!init_device_from_config(buf + SP_SDSC_DATA_OFFSET, config_num))
    {
      throw std::runtime_error(std::string("sp: error initializing the device"));
    }
  }
  else
  {
    // error, stop streaming, flush and retry
    throw std::runtime_error(std::string("sp: checksum or header invalid"));
  }
}

void SerialProtocolBase::read_data(uint8_t* buf, const uint8_t did)
{
  // check sensor existence to compute awaited len
  std::pair<SensorBase*, bool>* sensor = dev.get_sensor_by_idx(did);
  if (sensor)
  {
    if (sensor->second)  // active
    {
      size_t data_len = 0;
      size_t expected_len = SP_TIMESTAMP_LEN + sensor->first->get_len() + SP_CHKSUM_LEN;
      // check buffer size
      if (expected_len + SP_DATA_OFFSET < SP_MAX_BUF_SIZE)
      {
        // read additional data (timestamp + sensor len + checksum)
        data_len = s->readFrame(buf + SP_DATA_OFFSET, expected_len);
      }
      else
        throw std::runtime_error(std::string("sp:expects too much data for buffer max size"));
      // check received data
      if (data_len == expected_len)
      {
        // validate the whole message
        if (valid_checksum(buf, SP_DATA_OFFSET + expected_len))
        {
          // unpack the data
          if (!sensor->first->unpack(buf + SP_DATA_OFFSET))
            throw std::runtime_error(std::string("sp:sensor data storage not initialized"));
          // parse the data (sensor specific)
          sensor->first->parse();
        }
        else
          throw std::runtime_error(std::string("sp:data with invalid checksum"));
      }
      else
        throw std::runtime_error(std::string("sp:invalid data size"));
    }
    else
    {
      std::cerr << "sp:received data of an inactive sensor id " << (int)did << std::endl;
      throw std::runtime_error(std::string("sp:inactive sensor id"));
    }
  }
  else
  {
    // TODO we should maybe flush until we see a header ?
    std::cerr << "sp:found unknown sensor id " << (int)did << std::endl;
    throw std::runtime_error(std::string("sp:unknown sensor id"));
  }
}

void SerialProtocolBase::read_error(uint8_t* buf)
{
  size_t error_len = 0;
  error_len = s->readFrame(buf + SP_ERR_TYP_OFFSET, SP_ERR_TYP_LEN + SP_CHKSUM_LEN);
  if (error_len == SP_ERR_TYP_LEN + SP_CHKSUM_LEN)
  {
    uint8_t error_code = buf[SP_ERR_TYP_OFFSET];
    std::cerr << "sp: device sent an error:";
    switch (error_code)
    {
      case SP_ERR_CHKSUM:
        if (!valid_checksum(buf, 5))
          throw std::runtime_error(std::string("sp:err, chksum invalid checksum"));
        std::cerr << " device says checksum error " << std::endl;
        break;
      case SP_ERR_UNKNCMD:
        if (!valid_checksum(buf, 5))
          throw std::runtime_error(std::string("sp:err, cmd invalid checksum"));
        std::cerr << " device says unknown command " << std::endl;
        break;
      case SP_ERR_UNKNOWN:
        if (!valid_checksum(buf, 5))
          throw std::runtime_error(std::string("sp:err, unk invalid checksum"));
        std::cerr << " device encountered an unknown error" << std::endl;
        break;
      case SP_ERR_UNKNSID:
        if (!valid_checksum(buf, 5))
          throw std::runtime_error(std::string("sp:err, sid invalid checksum"));
        std::cerr << " device encountered an unknown sensor id" << std::endl;
        break;
      default:
        if (error_code > 0 && error_code < SP_ERR_STRMAX)
        {
          // read the error string
          char strbuf[error_code + 1];  // include null terminating char
          // strbuf[0] = (char)buf[ERROR_OFFSET+2];  // first string was already read
          // read str_len because one character was already read but checksum is still to be read
          uint8_t str_len = error_code;  // include the checksum
          s->readFrame(buf + SP_ERR_TYP_OFFSET + SP_ERR_TYP_LEN + SP_CHKSUM_LEN,
                       str_len);  // read the reminder of the string
          if (!valid_checksum(buf, SP_ERR_TYP_OFFSET + SP_ERR_TYP_LEN + str_len + SP_CHKSUM_LEN))
            throw std::runtime_error(std::string("sp:err, string invalid checksum"));
          memcpy(strbuf, buf + SP_ERR_TYP_OFFSET + SP_ERR_TYP_LEN, str_len);
          strbuf[str_len] = '\0';  // end of string
          std::cerr << " device reports error: " << strbuf << std::endl;
        }
        else
        {
          std::cerr << " device encountered an undocumented error" << std::endl;
        }
    }
  }
  else
  {
    throw std::runtime_error(std::string("sp:err, no error info"));
  }
}

void SerialProtocolBase::read()
{
  uint8_t read_buf[SP_MAX_BUF_SIZE];
  // init the first bytes to null to not see spurious values when debugging
  std::memset(read_buf, 0, 7);
  size_t read_buf_len = 0;
  try
  {
    // always read 3 bytes : HEADER + datagram id
    read_buf_len = s->readFrame(read_buf, SP_HEADER_LEN + SP_DID_LEN);
    if (read_buf_len == SP_HEADER_LEN + SP_DID_LEN)
    {
      if (verbose)
        std::cout << "sp: decoding answer" << std::endl;
      // check header
      if (!valid_header(read_buf))
      {
        if (verbose)
          std::cout << "sp: found header 0x" << std::hex << (int)read_buf[0] << " 0x" << std::hex << (int)read_buf[1]
                    << std::endl;
        throw std::runtime_error(std::string("sp:invalid header"));
      }
      // decode datagram ID
      uint8_t did = read_buf[SP_DID_OFFSET];
      // read additional data depending on the datagram id
      switch (did)
      {
        case SP_DID_DEVCNF:  // process master response
          if (verbose)
            std::cout << "sp: processing config answer" << std::endl;
          read_config(read_buf);
          break;
        case SP_DID_TOPOL:
          if (verbose)
            std::cerr << "sp: topology answer decoding not yet implemented" << std::endl;
          break;
        case SP_DID_SERIAL:
          if (verbose)
            std::cerr << "sp: serial answer decoding not yet implemented" << std::endl;
          break;
        case SP_DID_CALCNF:
          if (verbose)
            std::cerr << "sp: calib config answer decoding not yet implemented" << std::endl;
          break;
        case SP_DID_CALDSC:
          if (verbose)
            std::cerr << "sp: calib description answer decoding not yet implemented" << std::endl;
          break;
        case SP_DID_ERR:  // process error
          read_error(read_buf);
          break;
        case SP_DID_BCAST:
          break;
        default:
          if (did >= SP_DID_LID1 && did <= SP_DID_LIDMAX)  // process data
          {
            if (verbose)
              std::cout << "sp: processing data answer of sensor id " << did << std::endl;
            read_data(read_buf, did);
          }
          else  // process invalid datagram id
          {
            std::cerr << "sp:found datagram id 0x" << std::hex << (int)did << std::endl;
            throw std::runtime_error(std::string("sp:unknown datagram id"));
          }
      }
    }
    else
    {
      // nothing to await from sensor
      throw std::runtime_error(std::string("sp: nothing to read (did you start streaming ?)"));
    }
  }
  catch (const std::exception& e)
  {
    std::cerr << " " << e.what() << std::endl;
    std::cout << "sp: header 0x" << std::hex << (int)read_buf[0] << " 0x" << std::hex << (int)read_buf[1] << std::endl;
    std::cout << "sp: data 0x" << std::hex << (int)read_buf[2] << " 0x" << std::hex << (int)read_buf[3] << " 0x"
              << std::hex << (int)read_buf[4] << " 0x" << std::hex << (int)read_buf[5] << " 0x" << std::hex
              << (int)read_buf[6] << std::endl;

    throw std::runtime_error(std::string(""));
  }
}

bool SerialProtocolBase::get_data_as_float(float& val, const uint8_t did)
{
  // check sensor existence
  std::pair<SensorBase*, bool>* sensor = dev.get_sensor_by_idx(did);
  if (sensor)
  {
    if (sensor->first->get_len() >= 4)
    {
      val = *((float*)sensor->first->get_data());
      return true;
    }
  }
  return false;
}

bool SerialProtocolBase::get_data_as_3_float(float& x, float& y, float& z, const uint8_t did)
{
  // check sensor existence
  std::pair<SensorBase*, bool>* sensor = dev.get_sensor_by_idx(did);
  if (sensor)
  {
    if (sensor->first->get_len() >= 12)
    {
      uint8_t* buf = (uint8_t*)sensor->first->get_data();
      if (buf)
      {
        memcpy(&x, buf, sizeof(float));
        memcpy(&y, buf + 4, sizeof(float));
        memcpy(&z, buf + 8, sizeof(float));
        return true;
      }
    }
  }
  return false;
}

bool SerialProtocolBase::get_data_as_short(short& val, const uint8_t did)
{
  // check sensor existence
  std::pair<SensorBase*, bool>* sensor = dev.get_sensor_by_idx(did);
  if (sensor)
  {
    val = *((short*)sensor->first->get_data());
    return true;
  }
  return false;
}

bool SerialProtocolBase::get_data_as_unsigned_short(unsigned short& val, const uint8_t did)
{
  // check sensor existence
  std::pair<SensorBase*, bool>* sensor = dev.get_sensor_by_idx(did);
  if (sensor)
  {
    val = *((unsigned short*)sensor->first->get_data());
    return true;
  }
  return false;
}

uint32_t SerialProtocolBase::get_timestamp(const uint8_t did)
{
  // check sensor existence
  uint32_t t = 0;
  std::pair<SensorBase*, bool>* sensor = dev.get_sensor_by_idx(did);
  if (sensor)
  {
    t = sensor->first->get_timestamp();
  }
  return t;
}

/*
// REDO THIS
void SerialProtocolBase::process(uint8_t *buf, size_t buf_len)
{
  const std::vector<SensorBase*> sensors = get_sensors();
  // for each incoming message
  size_t index = 0;
  while (index < buf_len)
  {
    // red the datagram id
    size_t sen_id = (size_t)buf[index];
    dev.get_sensor_by_idx(sen_id);
  for (auto it=sensors.begin();
       it != sensors.end();
       it++)
  {

     for (auto it=sensors.begin(); it != sensors.end(); it++)
    {
      if (it.second.second)
        cached_max_stream_size += it.second.first.len;
    }

    size_t sen_len = it.len
    // read the correct amount of bytes
    try
    {
      buf_len = s->readFrame(buf, HEADER_LEN + 1 + sen_len);
    }
    catch (const std::exception &e) {
      std::cerr << " " <<  e.what() << std::endl;
      if (retry ==5)
        throw std::runtime_error(std::string("sp: device failed to answer"));
    }
    // check if no error in the return value

    it.parse(buf)

  }

  unpack_data(buf, len)
  }
}

void SerialProtocolBase::unpack_data(uint8_t *buf, size_t len)
{
  valid_header(buf);
  //valid_checksum();
}*/

void SerialProtocolBase::start_streaming(const uint8_t mode)
{
  uint8_t buf[5];
  uint32_t buf_len = 0;
  bool planned_streaming = false;
  switch (mode)
  {
    case SP_CMD_START_STREAM_CONT_ALL:
      planned_streaming = true;
      buf_len = gen_command(buf, SP_DID_MASTER, mode, 0);
      break;
    case SP_CMD_START_STREAM_CONT_SEL:
      planned_streaming = false;
    default:
      break;
  }
  if (buf_len)
  {
    try
    {
      send(buf, buf_len);
      streaming = planned_streaming;
    }
    catch (const std::exception& e)
    {
      std::cerr << e.what() << std::endl;
      throw std::runtime_error(std::string("sp: failed to start streaming"));
    }
  }
}

void SerialProtocolBase::stop_streaming()
{
  uint8_t buf[5];
  uint32_t buf_len;
  buf_len = gen_command(buf, SP_DID_MASTER, SP_CMD_STOP_STREAM, 0);
  try
  {
    send(buf, buf_len);
    streaming = false;
    s->flush();
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    throw std::runtime_error(std::string("sp: failed to stop streaming"));
  }
}

void SerialProtocolBase::send(const uint8_t* buf, const uint32_t len)
{
  try
  {
    s->writeFrame(buf, (size_t)len);
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    throw std::runtime_error(std::string("sp: failed to send"));
  }
}

uint32_t SerialProtocolBase::gen_command(uint8_t* buf, const uint8_t destination, const uint8_t command,
                                         const uint32_t size, const uint16_t stride, const uint8_t* data)
{
  // memset(buf, HEADER, HEADER_LEN * sizeof(uint8_t)); // cannot be done due to
  // little endianess
  memset(buf, SP_HDR1, sizeof(uint8_t));
  memset(buf + 1, SP_HDR2, sizeof(uint8_t));
  memset(buf + SP_DID_OFFSET, destination, sizeof(uint8_t));
  memset(buf + SP_CMD_OFFSET, command, sizeof(uint8_t));

  if (size && stride && data != NULL)
  {
    // stride is not sent, as it is implicit in the command
    memset(buf + SP_CMD_DATA_SZ_OFFSET, Lowbyte((uint16_t)size), sizeof(uint8_t));
    memset(buf + SP_CMD_DATA_SZ_OFFSET + 1, Highbyte((uint16_t)size), sizeof(uint8_t));
    // stride serves to copy the full data vector
    for (uint32_t i = 0; i < size * stride && i + SP_CMD_DATA_OFFSET < SP_MAX_BUF_SIZE; i++)
    {
      buf[i + SP_CMD_DATA_OFFSET] = data[i];
    }
  }
  uint32_t extra_size = (size > 0 ? SP_CMD_DATA_SZ_LEN : 0) + size * stride;
  buf[SP_CMD_DATA_SZ_OFFSET + extra_size] = compute_checksum(buf, SP_CMD_DATA_SZ_OFFSET + extra_size);
  return SP_CMD_DATA_SZ_OFFSET + extra_size + SP_CHKSUM_LEN;
}

uint32_t SerialProtocolBase::gen_master_config_req(uint8_t* buf)
{
  return gen_command(buf, SP_DID_MASTER, SP_CMD_CONFRQ, 0);
}

uint32_t SerialProtocolBase::gen_master_ping_req(uint8_t* buf)
{
  return gen_command(buf, SP_DID_MASTER, SP_CMD_ALIVE, 0);
}

uint32_t SerialProtocolBase::gen_master_trigger_req(uint8_t* buf)
{
  return gen_command(buf, SP_DID_MASTER, SP_CMD_START_STREAM_TRIG_ALL, 0);
}

uint32_t SerialProtocolBase::gen_sensor_trigger_req(uint8_t* buf, uint8_t sen_id)
{
  return gen_command(buf, sen_id, SP_CMD_START_STREAM_TRIG_SEL, 0);
}

uint32_t SerialProtocolBase::gen_topo_req(uint8_t* buf)
{
  return gen_command(buf, SP_DID_MASTER, SP_CMD_TOPORQ, 0);
}

uint32_t SerialProtocolBase::gen_serialnum_req(uint8_t* buf)
{
  return gen_command(buf, SP_DID_MASTER, SP_CMD_SERIAL, 0);
}

uint32_t SerialProtocolBase::gen_period_master_req(uint8_t* buf, const std::map<uint8_t, uint16_t>& period_map)
{
  /*for (std::map<int, MyClass>::iterator it = Map.begin(); it != Map.end();
  ++it)
  {
    it->second.Method();
  }*/
  uint16_t size = period_map.size();
  if (size)
  {
    uint8_t data[size * SP_PERIOD_DATA_LEN];
    uint16_t i = 0;
    for (const auto& item : period_map)
    {
      data[i * SP_PERIOD_DATA_LEN] = item.first;
      data[i * SP_PERIOD_DATA_LEN + 1] = Lowbyte(item.second);
      data[i * SP_PERIOD_DATA_LEN + 2] = Highbyte(item.second);
      i++;
    }
    return gen_command(buf, SP_DID_MASTER, SP_CMD_PERIOD, size, SP_PERIOD_DATA_LEN, data);
  }
  else
    return 0;
}

uint32_t SerialProtocolBase::gen_period_sensor_req(uint8_t* buf, const uint8_t sen_id, const uint16_t period)
{
  uint8_t data[SP_PERIOD_SENSOR_DATA_LEN];
  data[0] = Lowbyte(period);
  data[1] = Highbyte(period);
  return gen_command(buf, sen_id, SP_CMD_PERIOD, SP_PERIOD_SENSOR_DATA_LEN, 1, data);
}

/*
  uint8_t stopbuf[5] = {0xF0,0xC4, 0x00, 0x00, 0x34};
  //uint8_t stopbuf[5] = {0xF0,0xC4, 0x00, 0xC0, 0xF4};
  //uint8_t stopbuf[5] = {0xF0,0xC4, 0x00, 0xF1, 0xC5};
  //for (int i=0 ; i<20; ++i)
    s.writeFrame(stopbuf, 5);

  printf("sc: stopped all transmissions\n");
  try{
  read_len = s.readFrame(buf, 256); // read possibly incomplete frame
    }
  catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
  }
  printf("sc: reflushed %lu chars\n", read_len);

*/
}
