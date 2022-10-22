#ifndef _ROS_system_monitor_USBStatus_h
#define _ROS_system_monitor_USBStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace system_monitor
{

  class USBStatus : public ros::Msg
  {
    public:
      uint32_t usb_devices_length;
      typedef char* _usb_devices_type;
      _usb_devices_type st_usb_devices;
      _usb_devices_type * usb_devices;

    USBStatus():
      usb_devices_length(0), usb_devices(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->usb_devices_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->usb_devices_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->usb_devices_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->usb_devices_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->usb_devices_length);
      for( uint32_t i = 0; i < usb_devices_length; i++){
      uint32_t length_usb_devicesi = strlen(this->usb_devices[i]);
      varToArr(outbuffer + offset, length_usb_devicesi);
      offset += 4;
      memcpy(outbuffer + offset, this->usb_devices[i], length_usb_devicesi);
      offset += length_usb_devicesi;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t usb_devices_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      usb_devices_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      usb_devices_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      usb_devices_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->usb_devices_length);
      if(usb_devices_lengthT > usb_devices_length)
        this->usb_devices = (char**)realloc(this->usb_devices, usb_devices_lengthT * sizeof(char*));
      usb_devices_length = usb_devices_lengthT;
      for( uint32_t i = 0; i < usb_devices_length; i++){
      uint32_t length_st_usb_devices;
      arrToVar(length_st_usb_devices, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_usb_devices; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_usb_devices-1]=0;
      this->st_usb_devices = (char *)(inbuffer + offset-1);
      offset += length_st_usb_devices;
        memcpy( &(this->usb_devices[i]), &(this->st_usb_devices), sizeof(char*));
      }
     return offset;
    }

    const char * getType(){ return "system_monitor/USBStatus"; };
    const char * getMD5(){ return "e881e611f3b53172907c91e2b7d7f8ed"; };

  };

}
#endif