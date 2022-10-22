#ifndef _ROS_SERVICE_SetParam_h
#define _ROS_SERVICE_SetParam_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace configuration_node
{

static const char SETPARAM[] = "configuration_node/SetParam";

  class SetParamRequest : public ros::Msg
  {
    public:
      typedef const char* _nodeName_type;
      _nodeName_type nodeName;
      typedef const char* _param_type;
      _param_type param;
      typedef const char* _value_type;
      _value_type value;

    SetParamRequest():
      nodeName(""),
      param(""),
      value("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_nodeName = strlen(this->nodeName);
      varToArr(outbuffer + offset, length_nodeName);
      offset += 4;
      memcpy(outbuffer + offset, this->nodeName, length_nodeName);
      offset += length_nodeName;
      uint32_t length_param = strlen(this->param);
      varToArr(outbuffer + offset, length_param);
      offset += 4;
      memcpy(outbuffer + offset, this->param, length_param);
      offset += length_param;
      uint32_t length_value = strlen(this->value);
      varToArr(outbuffer + offset, length_value);
      offset += 4;
      memcpy(outbuffer + offset, this->value, length_value);
      offset += length_value;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_nodeName;
      arrToVar(length_nodeName, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_nodeName; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_nodeName-1]=0;
      this->nodeName = (char *)(inbuffer + offset-1);
      offset += length_nodeName;
      uint32_t length_param;
      arrToVar(length_param, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_param; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_param-1]=0;
      this->param = (char *)(inbuffer + offset-1);
      offset += length_param;
      uint32_t length_value;
      arrToVar(length_value, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_value; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_value-1]=0;
      this->value = (char *)(inbuffer + offset-1);
      offset += length_value;
     return offset;
    }

    const char * getType(){ return SETPARAM; };
    const char * getMD5(){ return "78cb7f395562d9ae65409e54f13fc0d0"; };

  };

  class SetParamResponse : public ros::Msg
  {
    public:
      typedef bool _res_type;
      _res_type res;

    SetParamResponse():
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

    const char * getType(){ return SETPARAM; };
    const char * getMD5(){ return "e27848a10f8e7e4030443887dfea101b"; };

  };

  class SetParam {
    public:
    typedef SetParamRequest Request;
    typedef SetParamResponse Response;
  };

}
#endif
