#ifndef _ROS_SERVICE_GetParams_h
#define _ROS_SERVICE_GetParams_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "configuration_node/Param.h"

namespace configuration_node
{

static const char GETPARAMS[] = "configuration_node/GetParams";

  class GetParamsRequest : public ros::Msg
  {
    public:
      typedef const char* _nodeName_type;
      _nodeName_type nodeName;

    GetParamsRequest():
      nodeName("")
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
     return offset;
    }

    const char * getType(){ return GETPARAMS; };
    const char * getMD5(){ return "fa90f177c62ffd4f5a57999c98f4104f"; };

  };

  class GetParamsResponse : public ros::Msg
  {
    public:
      uint32_t params_length;
      typedef configuration_node::Param _params_type;
      _params_type st_params;
      _params_type * params;

    GetParamsResponse():
      params_length(0), params(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->params_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->params_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->params_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->params_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->params_length);
      for( uint32_t i = 0; i < params_length; i++){
      offset += this->params[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t params_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      params_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      params_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      params_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->params_length);
      if(params_lengthT > params_length)
        this->params = (configuration_node::Param*)realloc(this->params, params_lengthT * sizeof(configuration_node::Param));
      params_length = params_lengthT;
      for( uint32_t i = 0; i < params_length; i++){
      offset += this->st_params.deserialize(inbuffer + offset);
        memcpy( &(this->params[i]), &(this->st_params), sizeof(configuration_node::Param));
      }
     return offset;
    }

    const char * getType(){ return GETPARAMS; };
    const char * getMD5(){ return "89842070ed1a0bb9e3a61b015ccb8285"; };

  };

  class GetParams {
    public:
    typedef GetParamsRequest Request;
    typedef GetParamsResponse Response;
  };

}
#endif
