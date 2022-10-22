#ifndef _ROS_system_monitor_HardwareStatus_h
#define _ROS_system_monitor_HardwareStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace system_monitor
{

  class HardwareStatus : public ros::Msg
  {
    public:
      typedef const char* _cs_name_type;
      _cs_name_type cs_name;
      typedef int16_t _battery_type;
      _battery_type battery;
      typedef const char* _batteryStatus_type;
      _batteryStatus_type batteryStatus;

    HardwareStatus():
      cs_name(""),
      battery(0),
      batteryStatus("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_cs_name = strlen(this->cs_name);
      varToArr(outbuffer + offset, length_cs_name);
      offset += 4;
      memcpy(outbuffer + offset, this->cs_name, length_cs_name);
      offset += length_cs_name;
      union {
        int16_t real;
        uint16_t base;
      } u_battery;
      u_battery.real = this->battery;
      *(outbuffer + offset + 0) = (u_battery.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_battery.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->battery);
      uint32_t length_batteryStatus = strlen(this->batteryStatus);
      varToArr(outbuffer + offset, length_batteryStatus);
      offset += 4;
      memcpy(outbuffer + offset, this->batteryStatus, length_batteryStatus);
      offset += length_batteryStatus;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_cs_name;
      arrToVar(length_cs_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_cs_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_cs_name-1]=0;
      this->cs_name = (char *)(inbuffer + offset-1);
      offset += length_cs_name;
      union {
        int16_t real;
        uint16_t base;
      } u_battery;
      u_battery.base = 0;
      u_battery.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_battery.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->battery = u_battery.real;
      offset += sizeof(this->battery);
      uint32_t length_batteryStatus;
      arrToVar(length_batteryStatus, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_batteryStatus; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_batteryStatus-1]=0;
      this->batteryStatus = (char *)(inbuffer + offset-1);
      offset += length_batteryStatus;
     return offset;
    }

    const char * getType(){ return "system_monitor/HardwareStatus"; };
    const char * getMD5(){ return "3991d6b77bc51cdca835482f159ac315"; };

  };

}
#endif