#ifndef SerialProtocolDefines_H
#define SerialProtocolDefines_H

#define SP_VERSION 0x01
#define SP_MAX_KNOWN_VERSION 0x01

// Basic Protocol Constants and Offsets
#define SP_HDR1 0xF0
#define SP_HDR2 0xC4
#define SP_HEADER 0xF0C4
#define SP_HEADER_LEN 2
#define SP_DID_OFFSET (SP_HEADER_LEN)
#define SP_DID_LEN 1
#define SP_TIMESTAMP_LEN 4
#define SP_DATA_OFFSET (SP_DID_OFFSET + SP_DID_LEN)
#define SP_ERR_TYP_OFFSET (SP_DID_OFFSET + SP_DID_LEN)
#define SP_ERR_TYP_LEN 1
#define SP_VERSION_OFFSET (SP_DID_OFFSET + SP_DID_LEN)
#define SP_VERSION_LEN 1
#define SP_DEVID_OFFSET (SP_VERSION_OFFSET + SP_VERSION_LEN)
#define SP_DEVID_LEN 1
#define SP_SDSC_OFFSET (SP_DEVID_OFFSET + SP_DEVID_LEN)
#define SP_SDSC_SZ_LEN 1
#define SP_SDSC_DATA_OFFSET (SP_SDSC_OFFSET + SP_SDSC_SZ_LEN)
#define SP_SDSC_SIZE 4
#define SP_CMD_OFFSET (SP_DID_OFFSET + SP_DID_LEN)
#define SP_CMD_LEN 1
#define SP_CHKSUM_CMD_OFFSET (SP_CMD_OFFSET + SP_CMD_LEN)
#define SP_CHKSUM_LEN 1
#define SP_CMD_DATA_SZ_OFFSET (SP_CMD_OFFSET + SP_CMD_LEN)
#define SP_CMD_DATA_SZ_LEN 2
#define SP_CMD_DATA_OFFSET (SP_CMD_DATA_SZ_OFFSET + SP_CMD_DATA_SZ_LEN)
#define SP_PING_LEN 4
//#define CONF_MIN_LEN (SDSC_OFFSET + 1)
#define SP_PERIOD_DATA_LEN 3
#define SP_PERIOD_SENSOR_DATA_LEN 2
#define SP_SEL_DATA_LEN 1
#define SP_ERR_LEN 4
#define SP_ERR_TXT_OFFSET 4
#define SP_OUT_LEN 10

// Common DatagramID
#define SP_DID_MASTER 0x00
#define SP_DID_DEVCNF 0x00
#define SP_DID_LID1 0x01
#define SP_DID_LIDMAX 0xC9
#define SP_DID_DEBUG 0xDB
#define SP_DID_ROM_R1 0xE0  // to be added to spec
#define SP_DID_ROM_R2 0xE1  // to be added to spec
#define SP_DID_PING 0xF0
#define SP_DID_TOPOL 0xF1   // moved from FD
#define SP_DID_SERIAL 0xF2  // moved from F5
#define SP_DID_CALCNF 0xF3
#define SP_DID_CALDSC 0xF4
#define SP_DID_LIDCNF 0xFC  // to be added to spec
#define SP_DID_ERR 0xFE
#define SP_DID_ALL 0xFF
#define SP_DID_BCAST 0xFF

// Command Types
#define SP_CMD_STOP_STREAM 0x00
#define SP_CMD_CONFRQ 0xC0
#define SP_CMD_TOPORQ 0xC1
#define SP_CMD_SERIAL 0xC2
#define SP_CMD_CALCNF 0xC3
#define SP_CMD_CALDSC 0xC4
#define SP_CMD_PERIOD 0xD0
#define SP_CMD_ROM_R1 0xE0
#define SP_CMD_ROM_R2 0xE1
#define SP_CMD_ROM_W2 0xEE
#define SP_CMD_ROM_W1 0xEF
#define SP_CMD_ALIVE 0xF0
#define SP_CMD_START_STREAM_CONT_ALL 0xF1
#define SP_CMD_START_STREAM_CONT_SEL 0xF2
#define SP_CMD_START_STREAM_TRIG_ALL 0xF3
#define SP_CMD_START_STREAM_TRIG_SEL 0xF4  // ???

#define SP_PERIOD_INACTIVE -1
#define SP_PERIOD_UNKNOWN_ID -2

// Error ID
#define SP_ERR_UNKNOWN 0x00
#define SP_ERR_CHKSUM 0xF0
#define SP_ERR_UNKNCMD 0xF1
#define SP_ERR_UNKNSID 0xF2

#define SP_ERR_STRMAX 0xEF

#endif /* SerialProtocolDefines_H */
