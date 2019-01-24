#include "serial_protocol.h"
#include "registered_sensors_impl.h"
#include "yaml-cpp/yaml.h"
#include <exception>
#include <stdexcept>
#include <cstring>
#include <string>
#include <iostream>
#include <utility>

namespace serial_protocol {

SensorBase::SensorBase(const unsigned int sen_len, const SensorType sensor_type) :
  sensor(sensor_type), len(0), dataptr(0), timestamp(0), previous_timestamp(0)
{
  // check data len consistency to registered len
  if (sensor_type.data_length > 0)
  {
    if (sen_len != sensor_type.data_length)
      throw std::runtime_error(std::string("sensor: non-matching data length"));
  }
  len = sen_len;
}

SensorBase::~SensorBase()
{
  if (dataptr)
    delete (uint8_t*)dataptr;
}

bool SensorBase::init()
{
  if (len)
  {
    if (dataptr!=0)
      delete (uint8_t*)dataptr;
    dataptr = (void*) new uint8_t[len];
    if (dataptr != 0)
      return true;
  }
  return false;
}

bool SensorBase::unpack(uint8_t *buf)
{
  // store the raw value, no processing at all
  if (!dataptr)
    return false;
  extract_timestamp(buf);
  std::memcpy((uint8_t *)dataptr, buf + TIMESTAMP_LEN, len);
  return true;
}

void* SensorBase::get_data()
{
  return dataptr;
}

unsigned int SensorBase::get_timestamp()
{
  return timestamp;
}

void SensorBase::extract_timestamp(uint8_t *buf)
{
  std::memcpy((uint8_t *)&timestamp, buf, TIMESTAMP_LEN);
  
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
  //Register("MPU9250", &Sensor_IMU_MPU9250::Create);
  Register("MPU9250_accelerometer", &Sensor_IMU_MPU9250_Acc::Create);
}

void SensorFactory::Register(const std::string & sensor_name, CreateSensorFn fnCreate)
{
  factory_map[sensor_name] = fnCreate;
}

SensorBase *SensorFactory::CreateSensor(const unsigned int sen_len, const SensorType sensor_type)
{
  SensorFactoryMap::iterator it = factory_map.find(sensor_type.name);
  if (it != factory_map.end() )
    return it->second(sen_len, sensor_type);
  return NULL;
}


/* *********************** */

Device::Device() 
{
  device.id = 0;
}

void Device::init(DeviceType dev_type)
{
  device = dev_type;
}

Device::~Device()
{
  std::vector< std::pair<SensorBase*, bool> >::iterator s;
  for (s=sensors.begin(); s!=sensors.end(); s++)
  {
    delete (*s).first;
  }
}

void Device::init_ros(ros::NodeHandle &nh)
{
  std::vector< std::pair<SensorBase*, bool> >::iterator s;
  for (s=sensors.begin(); s!=sensors.end(); s++)
  {
    SensorBase* sensor = (*s).first;
    sensor->init_ros(nh);
  }
}

std::string Device::get_serial()
{
  return serialnum;
}


std::pair<SensorBase*, bool> *Device::get_sensor_by_idx(size_t idx)
{
  int sensor_idx = idx - 1;
  if (sensor_idx >= 0 && sensor_idx < (int)sensors.size())
    return &sensors[sensor_idx];
  else
    return NULL;
}

void Device::add_sensor(unsigned int data_len, const SensorType sensor_type)
{
  try
  {
    SensorBase *sensor = SensorFactory::Get()->CreateSensor(data_len, sensor_type);
    
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
      throw std::runtime_error(std::string("sp: could not create a sensor"));
    }
  }
  catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    throw std::runtime_error(std::string("sp: could not add a sensor"));
  }
}

void Device::publish_all()
{
  std::vector< std::pair<SensorBase*, bool> >::iterator it;
  for (it=sensors.begin();
       it != sensors.end();
       it++)
  {
    std::pair<SensorBase*, bool> item = *it;
    if (item.second)
    {
      item.first->publish();
    }
  }
}

/* *********************** */


SerialProtocolBase::SerialProtocolBase(SerialCom *serial_com, 
  const std::string device_filename, const std::string sensor_filename) :
  verbose(false), streaming(false), s(serial_com), d_filename(device_filename),
  s_filename(sensor_filename)
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
  catch (const std::exception &e) {

    std::cerr << e.what() << std::endl;
    return false;
  }
}

#ifdef HAVE_ROS
void SerialProtocolBase::init_ros(ros::NodeHandle &nh)
{
  dev.init_ros(nh);
}
#endif

void SerialProtocolBase::config()
{
  uint8_t send_buf[MAX_BUF_SIZE];
  // request config
  size_t send_buf_len = gen_master_config_req(send_buf);
  try
  {
    //printf("sp: fake write\n");
    s->writeFrame(send_buf, send_buf_len);
  }
  catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    throw std::runtime_error(std::string("sp: failed to config"));
  }
  read();
}

bool SerialProtocolBase::init_device_from_config(uint8_t* buf, size_t config_num)
{
  for (size_t i=0; i < config_num; i++)
  {
    uint8_t* config_buf = buf + i * SDSC_SIZE * sizeof(uint8_t);
    unsigned int sen_id = config_buf[0] * 256 + config_buf[1];
    unsigned int sen_len = config_buf[2] * 256 + config_buf[3];
    if (verbose)
      printf("sp: for sensor %lu, found sen_id %u and sen_len %u\n", i, sen_id, sen_len);
    if (exists_sensor(sen_id))
    {
      try
      {
        dev.add_sensor(sen_len, sensor_types[sen_id]);
      }
      catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return false;
      }
    }
    else
      return false;
  }
  return true;
}

bool SerialProtocolBase::set_device(unsigned int dev_id)
{
  if(exists_device(dev_id))
  {
    dev.init(device_types[dev_id]);
    return true;
  }
  else
    return false;
}

bool SerialProtocolBase::valid_data(uint8_t* buf, size_t buf_len)
{
  return valid_header(buf) && valid_checksum(buf, buf_len);
}

bool SerialProtocolBase::valid_header(uint8_t* buf)
{
  return (uint16_t)(buf[0] * 256 + buf [1]) == HEADER;
}

bool SerialProtocolBase::valid_checksum(uint8_t* buf, size_t len)
{
  return compute_checksum(buf, len-1)==buf[len-1];
}

uint8_t SerialProtocolBase::compute_checksum(uint8_t* buf, size_t len)
{
  uint8_t xorsum = 0;
  for (size_t i=0; i < len; i++)  // do xor for each data byte
  {
    xorsum = xorsum ^ buf[i];
  }
  return xorsum;
}

void SerialProtocolBase::read_device_types(const unsigned int v)
{
  //read params for given version
  if (d_filename != "")
  {
    if (verbose)
      std::cout << "sp: reading device_types from " << d_filename << std::endl;
    //bool success = false;
    YAML::Node root = YAML::LoadFile(d_filename);
    if (root["agni_serial_protocol"])
    {
      YAML::Node agni_serial_protocol = root["agni_serial_protocol"];
      for(YAML::iterator it=agni_serial_protocol.begin(); it!=agni_serial_protocol.end(); ++it)
      {
        //std::string protocol_version_str = it->first.as<std::string>();
        // std::string protocol_version_str = 
        unsigned int protocol_version = it->first.as<unsigned int>();
        if (verbose)
          std::cout << "sp: found specs for version " <<  protocol_version << "\n";
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
            for(YAML::const_iterator devit=device_types_node.begin();devit!=device_types_node.end();++devit)
            {
              YAML::Node device_types_item = *devit;//->second;
              if (verbose)
                std::cout << "sp: extract device_type item ";
              DeviceType dt;
              std::stringstream sstr("");
              sstr << std::hex << device_types_item["type"].as<std::string>();
              sstr >> dt.id;
              dt.name = device_types_item["name"].as<std::string>();
              if (verbose)
                std::cout << dt.id <<" named: " << dt.name << "\n";
              dt.description = device_types_item["description"].as<std::string>();
              device_types[dt.id] = dt;
            }
            //success = true;
            return;
          }
          else
          {
            std::cerr << "sp: yaml does not contain specs for version " << v << "\n";
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

void SerialProtocolBase::read_sensor_types(const unsigned int v)
{
  //read params for given version
  if (s_filename != "")
  {
    if (verbose)
      std::cout << "sp: reading sensor_types from " << s_filename << std::endl;
    //bool success = false;
    YAML::Node root = YAML::LoadFile(s_filename);
    if (root["agni_serial_protocol"])
    {
      YAML::Node agni_serial_protocol = root["agni_serial_protocol"];
      for(YAML::iterator it=agni_serial_protocol.begin(); it!=agni_serial_protocol.end(); ++it)
      {
        unsigned int protocol_version = it->first.as<unsigned int>();
        if (verbose)
          std::cout << "sp: found specs for version " <<  protocol_version << "\n";
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
            for(YAML::const_iterator senit=sensor_types_node.begin();senit!=sensor_types_node.end();++senit)
            {
              YAML::Node sensor_types_item = *senit;//->second;
              if (verbose)
                std::cout << "sp: extract sensor_type item ";
              SensorType st;
              std::stringstream sstr("");
              sstr << std::hex << sensor_types_item["uid"].as<std::string>();
              sstr >> st.id;
              st.name = sensor_types_item["name"].as<std::string>();
              st.manufacturer = sensor_types_item["manufacturer"].as<std::string>();
              st.description = sensor_types_item["description"].as<std::string>();
              st.parser_library = sensor_types_item["parser_library"].as<std::string>();
              st.data_length = sensor_types_item["data_length"].as<unsigned int>();
              if (verbose)
                std::cout << st.id << " named: " << st.name << " with length " << st.data_length << "\n";
              sensor_types[st.id] = st;
            }
            //success = true;
            return;
          }
          else
          {
            std::cerr << "sp: yaml does not contain specs for version " << v << "\n";
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

bool SerialProtocolBase::exists_device(unsigned int dev_id)
{
  return device_types.find(dev_id)!=device_types.end();
}

bool SerialProtocolBase::exists_sensor(unsigned int sen_id)
{
   return sensor_types.find(sen_id)!=sensor_types.end();
}

DeviceType SerialProtocolBase::get_device(){return dev.device;}

void SerialProtocolBase::trigger(const unsigned int mode, const unsigned int sen_id)
{
  if (!streaming)
  {
    uint8_t send_buf[5];
    size_t send_buf_len = 0;
    if (sen_id == 0)
    {
      send_buf_len=gen_master_trigger_req(send_buf);
    }
    else
    {
      // check if sensor exists
      if (exists_sensor(sen_id))
      {
        send_buf_len=gen_sensor_trigger_req(send_buf, sen_id);
      }
    }
    if (send_buf_len)
    {
      try
      {
        send(send_buf, send_buf_len);
      }
      catch (const std::exception &e) {
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
  catch (const std::exception &e) {
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

void SerialProtocolBase::read_config(uint8_t *buf)
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
  buf_len = s->readFrame(buf+VERSION_OFFSET, 3);
  if (buf_len < 3)
  {
    std::cerr << "sp: incorrect config header size: " <<  buf_len << " < " << 3 << "\n";
    throw std::runtime_error(std::string("sp: device did not answer enough data"));
  }
  // read versions
  int version = (int)buf[VERSION_OFFSET];
  if (version > 0 and version <= MAX_KNOWN_VERSION)
  {
    read_device_types(version);
    read_sensor_types(version);
  }
  else
  {
    std::cerr << "sp: found unknown version (" <<  "0x" << std::uppercase << std::hex << version << ")\n";
    throw std::runtime_error(std::string("sp: unsupported version"));
  }

  // validate master is answering
  if (buf[DID_OFFSET] != DID_MASTER)
  {
    throw std::runtime_error(std::string("sp: config answer from non-master not supported"));
  }

  // create a device
  if (!set_device(buf[DEVID_OFFSET]))
  {
    throw std::runtime_error(std::string("sp: device exists already"));
  }

  // get how much more config comes
  size_t config_num = (size_t)buf[SDSC_OFFSET];
  // read next part of the config message
  size_t config_len = 0;
  try
  {
    config_len = s->readFrame(buf + SDSC_OFFSET + 1, config_num * SDSC_SIZE + 1);
  }
  catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    throw std::runtime_error(std::string("sp: device failed to answer"));
  }
  // check data
  if (config_len !=  config_num * SDSC_SIZE + 1)
  {
    std::cerr << "sp: read " << config_len << " bytes of config_len instead of " << config_num * SDSC_SIZE + 1 << std::endl;
    throw std::runtime_error(std::string("sp: incomplete sensor description datagram"));
  }
  // validate the full frame
  if (valid_data(buf, SDSC_OFFSET + 1 + config_len))
  {
    if (!init_device_from_config(buf + SDSC_OFFSET + 1, config_num))
    {
      throw std::runtime_error(std::string("sp: error initializing the device"));
    }
  }
  else
  {
    //error, stop streaming, flush and retry
    throw std::runtime_error(std::string("sp: checksum or header invalid"));
  }
}

void SerialProtocolBase::read_data(uint8_t *buf, size_t did)
{
  // check sensor existence to compute awaited len
  std::pair<SensorBase*, bool> *sensor = dev.get_sensor_by_idx(did);
  if (sensor)
  {
    if (sensor->second)  // active
    {
      size_t data_len = 0;
      size_t expected_len = TIMESTAMP_LEN + sensor->first->get_len() + 1;
      // check buffer size
      if (expected_len + DATA_OFFSET < MAX_BUF_SIZE)
      {
        // read additional data (timestamp + sensor len + checksum)
        data_len = s->readFrame(buf + DATA_OFFSET, expected_len);
      }
      else
        throw std::runtime_error(std::string("sp:expects too much data for buffer max size"));
      // check received data
      if (data_len == expected_len)
      {
        // validate the whole message
        if (valid_checksum(buf, DATA_OFFSET + expected_len))
        {
          // unpack the data
          if(!sensor->first->unpack(buf + DATA_OFFSET))
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

void SerialProtocolBase::read_error(uint8_t *buf)
{
  size_t error_len = 0;
  error_len = s->readFrame(buf + ERROR_OFFSET, 2);
  if (error_len==2)
  {
    uint8_t error_code=buf[ERROR_OFFSET];
    std::cerr << "sp: device sent an error:";
    switch(error_code)
    {
      case ERR_CHKSUM:
        if (!valid_checksum(buf, 5))
          throw std::runtime_error(std::string("sp:err, chksum invalid checksum"));
        std::cerr << " device says checksum error " << std::endl;
        break;
      case ERR_UNKCMD:
        if (!valid_checksum(buf, 5))
          throw std::runtime_error(std::string("sp:err, cmd invalid checksum"));
        std::cerr << " device says unknown command " << std::endl;
        break;
      case ERR_UNKNOWN:
        if (!valid_checksum(buf, 5))
          throw std::runtime_error(std::string("sp:err, unk invalid checksum"));
        std::cerr << " device encountered an unknown error" << std::endl;
        break;
      default:
        if ( error_code > 0 && error_code < ERR_STRMAXLEN )
        {
          // read the error string
          size_t str_len = error_code;  // include the checksum
          char* strbuf[error_code+1];  // include null terminating char
          //strbuf[0] = (char)buf[ERROR_OFFSET+2];  // first string was already read
          s->readFrame(buf+ERROR_OFFSET+2, str_len);  // read the reminder of the string
          if (!valid_checksum(buf, ERROR_OFFSET+1+str_len+1))
            throw std::runtime_error(std::string("sp:err, string invalid checksum"));
          memcpy(strbuf, buf+ERROR_OFFSET+1, str_len);
          strbuf[str_len] = '\0'; // end of string
          std::cerr << strbuf << std::endl;
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
  uint8_t read_buf[MAX_BUF_SIZE];
  size_t read_buf_len = 0;
  try
  {
    // always read 3 bytes HEADER + datagram id
    read_buf_len = s->readFrame(read_buf, DATA_OFFSET);
    if(read_buf_len == DATA_OFFSET)
    {
      if (verbose)
        std::cout << "sp: decoding answer" << std::endl;
      // check header
      if (!valid_header(read_buf))
        throw std::runtime_error(std::string("sp:invalid header"));
      // decode datagram ID
      uint8_t did = read_buf[DID_OFFSET];
      // read additional data depending on the datagram id
      switch(did)
      {
        case DID_MASTER:  // process master response
          if (verbose)
            std::cout << "sp: processing config answer" << std::endl;
          read_config(read_buf);
          break;
        case DID_ERROR:  // process error
          read_error(read_buf);
          break;
        case DID_SERIAL:
          break;
        case DID_TOPO:
          break;
        case DID_BCAST:
          break;
        default:
          if (did >0 && did <= DID_SEN_MAX)  // process data
          {
            if (verbose)
              std::cout << "sp: processing data answer" << std::endl;
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
  catch (const std::exception &e) {
      std::cerr << " " <<  e.what() << std::endl;
      throw std::runtime_error(std::string(""));
  }
}

bool SerialProtocolBase::get_data_as_float(float &val, unsigned int did)
{
  // check sensor existence
  std::pair<SensorBase*, bool> *sensor = dev.get_sensor_by_idx(did);
  if (sensor)
  {
    if(sensor->first->get_len() >=4)
    {
      val = *((float*)sensor->first->get_data());
      return true;
    }
  }
  return false;
}

bool SerialProtocolBase::get_data_as_3_float(float &x, float &y, float &z, unsigned int did)
{
  // check sensor existence
  std::pair<SensorBase*, bool> *sensor = dev.get_sensor_by_idx(did);
  if (sensor)
  {
    if(sensor->first->get_len() >=12)
    {
      uint8_t* buf = (uint8_t*)sensor->first->get_data();
      if (buf)
      {
        memcpy(&x, buf, sizeof(float));
        memcpy(&y, buf+4, sizeof(float));
        memcpy(&z, buf+8, sizeof(float));
        return true;
      }
    }
  }
  return false;
}

bool SerialProtocolBase::get_data_as_short(short &val, unsigned int did)
{
  // check sensor existence
  std::pair<SensorBase*, bool> *sensor = dev.get_sensor_by_idx(did);
  if (sensor)
  {
    val = *((short*)sensor->first->get_data());
    return true;
  }
  return false;
}

bool SerialProtocolBase::get_data_as_unsigned_short(unsigned short &val, unsigned int did)
{
  // check sensor existence
  std::pair<SensorBase*, bool> *sensor = dev.get_sensor_by_idx(did);
  if (sensor)
  {
    val = *((unsigned short*)sensor->first->get_data());
    return true;
  }
  return false;
}

unsigned int SerialProtocolBase::get_timestamp(unsigned int did)
{
  // check sensor existence
  unsigned int t = 0;
  std::pair<SensorBase*, bool> *sensor = dev.get_sensor_by_idx(did);
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

void SerialProtocolBase::start_streaming(const unsigned int mode)
{
  uint8_t buf[5];
  size_t buf_len = 0;
  bool planned_streaming = false;
  switch (mode)
  {
    case CMD_START_STREAM_CONT_ALL:
      planned_streaming = true;
      buf_len=gen_command(buf, DID_MASTER, mode, 0);
      break;
    case CMD_START_STREAM_CONT_SEL:
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
    catch (const std::exception &e) {
      std::cerr << e.what() << std::endl;
      throw std::runtime_error(std::string("sp: failed to start streaming"));
    }   
  }
}

void SerialProtocolBase::stop_streaming()
{
  uint8_t buf[5];
  size_t buf_len;
  buf_len=gen_command(buf, DID_MASTER, CMD_STOP_STREAM, 0);
  try
  {
    send(buf, buf_len);
    streaming = false;
    s->flush();
  }
  catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    throw std::runtime_error(std::string("sp: failed to stop streaming"));
  }
}

void SerialProtocolBase::send(uint8_t *buf, size_t len)
{
  try
  {
    s->writeFrame(buf, len);
  }
  catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    throw std::runtime_error(std::string("sp: failed to send"));
  }
}

size_t SerialProtocolBase::gen_command(uint8_t *buf, uint8_t destination, uint8_t command, size_t size, uint8_t *data)
{
  // memset(buf, HEADER, HEADER_LEN * sizeof(uint8_t)); // cannot be done due to little endianess
  memset(buf, HDR1, sizeof(uint8_t)); 
  memset(buf+1, HDR2, sizeof(uint8_t)); 
  memset(buf + DID_OFFSET, destination, sizeof(uint8_t));
  memset(buf + CMD_OFFSET, command, sizeof(uint8_t));

  if (size && data != NULL)
  {
    memset(buf+CMD_DATA_SZ_OFFSET, size, 1*sizeof(uint8_t));
    for (unsigned int i=0; i< size && i + CMD_DATA_OFFSET < MAX_BUF_SIZE; i++)
    {
      buf[i+CMD_DATA_OFFSET] = data[i];
    }
  }
  unsigned int extra_size = (size>0?1:0) + size;
  buf[CMD_DATA_SZ_OFFSET + extra_size] = compute_checksum(buf, CMD_DATA_SZ_OFFSET + extra_size);
  return CMD_DATA_SZ_OFFSET + extra_size + 1;
}

size_t SerialProtocolBase::gen_master_config_req(uint8_t *buf)
{
  return gen_command(buf, DID_MASTER, CMD_CONFRQ, 0);
}

size_t SerialProtocolBase::gen_master_ping_req(uint8_t *buf)
{
  return gen_command(buf, DID_MASTER, CMD_ALIVE, 0);
}

size_t SerialProtocolBase::gen_master_trigger_req(uint8_t *buf)
{
  return gen_command(buf, DID_MASTER, CMD_TRIG, 0);
}

size_t SerialProtocolBase::gen_sensor_trigger_req(uint8_t *buf, const unsigned int sen_id)
{
  return gen_command(buf, (uint8_t)sen_id, CMD_TRIG, 0);
}

size_t SerialProtocolBase::gen_topo_req(uint8_t *buf)
{
  return gen_command(buf, DID_MASTER, CMD_TOPORQ, 0);
}

size_t SerialProtocolBase::gen_serialnum_req(uint8_t *buf)
{
  return gen_command(buf, DID_MASTER, CMD_SERIAL, 0);
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
