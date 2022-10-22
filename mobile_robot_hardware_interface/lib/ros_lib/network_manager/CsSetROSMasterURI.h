#ifndef _ROS_SERVICE_CsSetROSMasterURI_h
#define _ROS_SERVICE_CsSetROSMasterURI_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace network_manager
{

static const char CSSETROSMASTERURI[] = "network_manager/CsSetROSMasterURI";

  class CsSetROSMasterURIRequest : public ros::Msg
  {
    public:
      typedef const char* _ros_master_uri_type;
      _ros_master_uri_type ros_master_uri;

    CsSetROSMasterURIRequest():
      ros_master_uri("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_ros_master_uri = strlen(this->ros_master_uri);
      varToArr(outbuffer + offset, length_ros_master_uri);
      offset += 4;
      memcpy(outbuffer + offset, this->ros_master_uri, length_ros_master_uri);
      offset += length_ros_master_uri;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_ros_master_uri;
      arrToVar(length_ros_master_uri, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_ros_master_uri; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_ros_master_uri-1]=0;
      this->ros_master_uri = (char *)(inbuffer + offset-1);
      offset += length_ros_master_uri;
     return offset;
    }

    const char * getType(){ return CSSETROSMASTERURI; };
    const char * getMD5(){ return "0084e2feab4ddb262c6564c2d392a692"; };

  };

  class CsSetROSMasterURIResponse : public ros::Msg
  {
    public:
      typedef bool _res_type;
      _res_type res;

    CsSetROSMasterURIResponse():
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

    const char * getType(){ return CSSETROSMASTERURI; };
    const char * getMD5(){ return "e27848a10f8e7e4030443887dfea101b"; };

  };

  class CsSetROSMasterURI {
    public:
    typedef CsSetROSMasterURIRequest Request;
    typedef CsSetROSMasterURIResponse Response;
  };

}
#endif
