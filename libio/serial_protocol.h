#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H
#include <string>
#include <vector>
#include <map>
#include <utility>
#include <stdint.h>
#include "serial_com.h"

// protocol constants
#define HEADER 0xF0C4
#define HDR1  0xF0
#define HDR2  0xC4
#define HEADER_LEN 2
#define DID_OFFSET  (HEADER_LEN)
#define VERSION_OFFSET  (DID_OFFSET+1)
#define DEVID_OFFSET  (VERSION_OFFSET+1)
#define SDSC_OFFSET (DEVID_OFFSET+1)
#define SDSC_SIZE 4
#define CONF_MIN_LEN  (SDSC_OFFSET+1)
#define TIMESTAMP_LEN 4
#define DATA_OFFSET (DID_OFFSET+1)
#define ERROR_OFFSET (DID_OFFSET+1)

#define MAX_BUF_SIZE 50
#define CMD_OFFSET  (DID_OFFSET+1)
#define CMD_DATA_SZ_OFFSET (CMD_OFFSET+1)
#define CMD_DATA_OFFSET (CMD_DATA_SZ_OFFSET+1)

#define DID_MASTER  0x00
#define DID_SEN_MAX 0xC9
#define DID_SERIAL  0xF5
#define DID_TOPO    0xFD
#define DID_ERROR   0xFE
#define DID_BCAST   0xFF

#define CMD_STOP_STREAM       0x00
#define CMD_CONFRQ  0xC0
#define CMD_TOPORQ  0xC1
#define CMD_SERIAL  0xC2
#define CMD_PERIOD  0xD0
#define CMD_ROM_R1  0xE0
#define CMD_ROM_R2  0xE1
#define CMD_ROM_W2  0xEE
#define CMD_ROM_W1  0xEF
#define CMD_ALIVE   0xF0
#define CMD_START_STREAM_CONT_ALL 0xF1
#define CMD_START_STREAM_CONT_SEL 0xF2
#define CMD_TRIG 0xF3

#define ERR_UNKNOWN 0x00
#define ERR_CHKSUM  0xF0
#define ERR_UNKCMD  0xF1
#define ERR_STRMAXLEN   0xEF

#define MAX_KNOWN_VERSION 1

namespace serial_protocol {

struct DeviceType
{
  unsigned int id;
  std::string name;
  std::string description;
};

struct SensorType
{
  unsigned int id;
  std::string name;
  std::string description;
  std::string parser_library;
  unsigned int data_length;
};


class SensorBase {
public:
  explicit SensorBase(const unsigned int sen_len, const SensorType sensor_type);
  virtual ~SensorBase();
  virtual bool init() = 0;
  virtual bool parse(uint8_t *buf) = 0;
  void* get_data();
  unsigned int get_timestamp();
  unsigned int get_len() { return len; }

protected:
  void extract_timestamp(uint8_t *buf);
  void* dataptr;
  unsigned int timestamp;
  SensorType sensor;
  unsigned int len;
};

typedef SensorBase* (*CreateSensorFn)(const unsigned int sen_len, const SensorType sensor_type);

/* **********************
 * 
 * based on Factory Pattern in C++
 * Cale Dunlap, 15 Sep 2012 
 * 
 */

class SensorFactory {
private:
  SensorFactory(const SensorFactory &);
  SensorFactory();
  SensorFactory &operator=(const SensorFactory &) { return *this; }
  
  typedef std::map<std::string, CreateSensorFn> SensorFactoryMap;
  SensorFactoryMap factory_map;
public:
  ~SensorFactory() { factory_map.clear(); }
  
  static SensorFactory *Get()
  {
    static SensorFactory instance;
    return &instance;
  }
  
  void Register(const std::string &sensor_name, CreateSensorFn fnCreate);
  SensorBase *CreateSensor(const unsigned int sen_len, const SensorType sensor_type);
};


class Device
{
public:
  Device();
  ~Device();
  void init(DeviceType dev_type);
  std::string get_serial();

  std::vector< std::pair<SensorBase*, bool> > &get_sensors()
  {
    return sensors;
  }
  std::pair<SensorBase*, bool> *get_sensor_by_idx(size_t idx);
  void add_sensor(unsigned int data_len, SensorType sensor_type);
  void process_data(uint8_t *buf, size_t len);

  DeviceType device;
protected:
  
  std::vector< std::pair<SensorBase*, bool> > sensors;
  //std::vector<bool> active_sensors;
  size_t cached_max_stream_size;
  bool active_sensor_modified;
  std::string serialnum;
};

class SerialProtocolBase
{
public:
  explicit SerialProtocolBase(SerialCom *serial_com);
  ~SerialProtocolBase();
  bool init();
  bool set_device(unsigned int dev_id);
  void start_streaming(const unsigned int mode=CMD_START_STREAM_CONT_ALL);
  void loop();
  //void process();
  double get_data_as_double(unsigned int did);
  unsigned int get_timestamp(unsigned int did);
  void trigger(const unsigned int mode, const unsigned int sen_id);
  void stop_streaming();

  void read_device_types(std::string filename);
  void read_sensor_types(std::string filename);
  bool exists_device(unsigned int dev_id);
  bool exists_sensor(unsigned int sen_id);

  DeviceType get_device();
  bool verbose;

protected:
  void config();
  void read_config(uint8_t *buf);
  void read_error(uint8_t *buf);
  void read_data(uint8_t *buf, size_t did);
  void read();

  
  //void unpack_data(uint8_t *buf);
  bool valid_data(uint8_t* buf, size_t buf_len);
  bool init_device_from_config(uint8_t* buf, size_t config_num);

  bool valid_header(uint8_t* buf);
  bool valid_checksum(uint8_t* buf, size_t len);
  uint8_t compute_checksum(uint8_t* buf, size_t len);

  void send(uint8_t *buf, size_t len);
  size_t gen_command(uint8_t *buf, uint8_t destination, uint8_t command, size_t size, uint8_t *data=NULL);
  size_t gen_master_ping_req(uint8_t *buf);
  size_t gen_master_config_req(uint8_t *buf);
  size_t gen_sensor_trigger_req(uint8_t *buf, const unsigned int sen_id);
  size_t gen_master_trigger_req(uint8_t *buf);
  size_t gen_topo_req(uint8_t *buf);
  size_t gen_serialnum_req(uint8_t *buf);

  unsigned int version;
  bool streaming;
  std::map<unsigned int, DeviceType> device_types;
  std::map<unsigned int, SensorType> sensor_types;
  SerialCom *s;
  Device dev;
};

}
#endif
