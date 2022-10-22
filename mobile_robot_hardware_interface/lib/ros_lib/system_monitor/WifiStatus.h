#ifndef _ROS_system_monitor_WifiStatus_h
#define _ROS_system_monitor_WifiStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace system_monitor
{

  class WifiStatus : public ros::Msg
  {
    public:
      typedef const char* _ssid_type;
      _ssid_type ssid;
      typedef const char* _ip_type;
      _ip_type ip;
      typedef const char* _mac_type;
      _mac_type mac;

    WifiStatus():
      ssid(""),
      ip(""),
      mac("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_ssid = strlen(this->ssid);
      varToArr(outbuffer + offset, length_ssid);
      offset += 4;
      memcpy(outbuffer + offset, this->ssid, length_ssid);
      offset += length_ssid;
      uint32_t length_ip = strlen(this->ip);
      varToArr(outbuffer + offset, length_ip);
      offset += 4;
      memcpy(outbuffer + offset, this->ip, length_ip);
      offset += length_ip;
      uint32_t length_mac = strlen(this->mac);
      varToArr(outbuffer + offset, length_mac);
      offset += 4;
      memcpy(outbuffer + offset, this->mac, length_mac);
      offset += length_mac;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_ssid;
      arrToVar(length_ssid, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_ssid; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_ssid-1]=0;
      this->ssid = (char *)(inbuffer + offset-1);
      offset += length_ssid;
      uint32_t length_ip;
      arrToVar(length_ip, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_ip; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_ip-1]=0;
      this->ip = (char *)(inbuffer + offset-1);
      offset += length_ip;
      uint32_t length_mac;
      arrToVar(length_mac, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_mac; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_mac-1]=0;
      this->mac = (char *)(inbuffer + offset-1);
      offset += length_mac;
     return offset;
    }

    const char * getType(){ return "system_monitor/WifiStatus"; };
    const char * getMD5(){ return "a2691b9d313b84905e0080c7d7959257"; };

  };

}
#endif