#ifndef _ROS_SERVICE_SetROSMasterURI_h
#define _ROS_SERVICE_SetROSMasterURI_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace configuration_node
{

static const char SETROSMASTERURI[] = "configuration_node/SetROSMasterURI";

  class SetROSMasterURIRequest : public ros::Msg
  {
    public:
      typedef const char* _uri_type;
      _uri_type uri;

    SetROSMasterURIRequest():
      uri("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_uri = strlen(this->uri);
      varToArr(outbuffer + offset, length_uri);
      offset += 4;
      memcpy(outbuffer + offset, this->uri, length_uri);
      offset += length_uri;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_uri;
      arrToVar(length_uri, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_uri; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_uri-1]=0;
      this->uri = (char *)(inbuffer + offset-1);
      offset += length_uri;
     return offset;
    }

    const char * getType(){ return SETROSMASTERURI; };
    const char * getMD5(){ return "636fe5e07550f026d28388e95c38a9f4"; };

  };

  class SetROSMasterURIResponse : public ros::Msg
  {
    public:
      typedef bool _res_type;
      _res_type res;

    SetROSMasterURIResponse():
      res(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_res;
      u_res.real = this->res;
      *(outbuffer + offset + 0) = (u_res.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->res);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_res;
      u_res.base = 0;
      u_res.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->res = u_res.real;
      offset += sizeof(this->res);
     return offset;
    }

    const char * getType(){ return SETROSMASTERURI; };
    const char * getMD5(){ return "e27848a10f8e7e4030443887dfea101b"; };

  };

  class SetROSMasterURI {
    public:
    typedef SetROSMasterURIRequest Request;
    typedef SetROSMasterURIResponse Response;
  };

}
#endif
