#ifndef _ROS_SERVICE_StopTracking_h
#define _ROS_SERVICE_StopTracking_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace realsense_person
{

static const char STOPTRACKING[] = "realsense_person/StopTracking";

  class StopTrackingRequest : public ros::Msg
  {
    public:

    StopTrackingRequest()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return STOPTRACKING; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class StopTrackingResponse : public ros::Msg
  {
    public:
      typedef int32_t _status_type;
      _status_type status;
      typedef const char* _status_desc_type;
      _status_desc_type status_desc;

    StopTrackingResponse():
      status(0),
      status_desc("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_status;
      u_status.real = this->status;
      *(outbuffer + offset + 0) = (u_status.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_status.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_status.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_status.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->status);
      uint32_t length_status_desc = strlen(this->status_desc);
      varToArr(outbuffer + offset, length_status_desc);
      offset += 4;
      memcpy(outbuffer + offset, this->status_desc, length_status_desc);
      offset += length_status_desc;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_status;
      u_status.base = 0;
      u_status.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_status.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_status.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_status.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->status = u_status.real;
      offset += sizeof(this->status);
      uint32_t length_status_desc;
      arrToVar(length_status_desc, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_status_desc; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_status_desc-1]=0;
      this->status_desc = (char *)(inbuffer + offset-1);
      offset += length_status_desc;
     return offset;
    }

    const char * getType(){ return STOPTRACKING; };
    const char * getMD5(){ return "dbfa2262d85d80c0b56ce4d61f4542fd"; };

  };

  class StopTracking {
    public:
    typedef StopTrackingRequest Request;
    typedef StopTrackingResponse Response;
  };

}
#endif
